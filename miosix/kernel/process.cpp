/***************************************************************************
 *   Copyright (C) 2012 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#include <stdexcept>
#include <memory>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <algorithm>
#include <sys/wait.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <limits.h>

#include "sync.h"
#include "process_pool.h"
#include "process.h"

using namespace std;

#ifdef WITH_PROCESSES

namespace miosix {

/**
 * Used to check if a pointer passed from userspace is aligned
 */
static bool aligned(void *x) { return (reinterpret_cast<unsigned>(x) & 0b11)==0; }

/**
 * This class contains information on all the processes in the system
 */
class Processes
{
public:
    /**
     * \return the instance of this class (singleton)
     */
    static Processes& instance();
    
    ///Used to assign a new pid to a process
    pid_t pidCounter;
    ///Maps the pid to the Process instance. Includes zombie processes
    map<pid_t,ProcessBase *> processes;
    ///Uset to guard access to processes and pidCounter
    Mutex procMutex;
    ///Used to wait on process termination
    ConditionVariable genericWaiting;
    
private:
    Processes()
    {
        ProcessBase *kernel=Thread::getCurrentThread()->getProcess();
        assert(kernel->getPid()==0);
        processes[0]=kernel;
    }
    Processes(const Processes&)=delete;
    Processes& operator=(const Processes&)=delete;
};

Processes& Processes::instance()
{
    static Processes singleton;
    return singleton;
}

//
// class Process
//

pid_t Process::create(const ElfProgram& program)
{
    Processes& p=Processes::instance();
    ProcessBase *parent=Thread::getCurrentThread()->proc;
    unique_ptr<Process> proc(new Process(program));
    {   
        Lock<Mutex> l(p.procMutex);
        proc->pid=getNewPid();
        proc->ppid=parent->pid;
        parent->childs.push_back(proc.get());
        p.processes[proc->pid]=proc.get();
    }
    auto thr=Thread::createUserspace(Process::start,0,Thread::DEFAULT,proc.get());
    if(thr==nullptr)
    {
        Lock<Mutex> l(p.procMutex);
        p.processes.erase(proc->pid);
        parent->childs.remove(proc.get());
        throw runtime_error("Thread creation failed");
    }
    //Cannot throw bad_alloc due to the reserve in Process's constructor.
    //This ensures we will never be in the uncomfortable situation where a
    //thread has already been created but there's no memory to list it
    //among the threads of a process
    proc->threads.push_back(thr);
    thr->wakeup(); //Actually start the thread, now that everything is set up
    pid_t result=proc->pid;
    proc.release(); //Do not delete the pointer
    return result;
}

pid_t Process::spawn(const char *path)
{
    if(path==nullptr || path[0]=='\0') return -EFAULT;
    string filePath=path; //TODO: expand ./program using cwd of correct file descriptor table
    ResolvedPath openData=FilesystemManager::instance().resolvePath(filePath);
    if(openData.result<0) return -ENOENT;
    StringPart relativePath(filePath,string::npos,openData.off);
    intrusive_ref_ptr<FileBase> file;
    if(int res=openData.fs->open(file,relativePath,O_RDONLY,0)<0) return res;
    MemoryMappedFile mmFile=file->getFileFromMemory();
    if(mmFile.isValid()==false) return -EFAULT;
    if(reinterpret_cast<unsigned int>(mmFile.data) & 0x3) return -ENOEXEC;
    ElfProgram prog(reinterpret_cast<const unsigned int*>(mmFile.data),mmFile.size);
    return Process::create(prog);
}

pid_t Process::getppid(pid_t proc)
{
    Processes& p=Processes::instance();
    Lock<Mutex> l(p.procMutex);
    map<pid_t,ProcessBase *>::iterator it=p.processes.find(proc);
    if(it==p.processes.end()) return -1;
    return it->second->ppid;
}

pid_t Process::waitpid(pid_t pid, int* exit, int options)
{
    Processes& p=Processes::instance();
    Lock<Mutex> l(p.procMutex);
    ProcessBase *self=Thread::getCurrentThread()->proc;
    if(pid<=0)
    {
        //Wait for a generic child process
        if(self->zombies.empty() && (options & WNOHANG)) return 0;
        while(self->zombies.empty())
        {
            if(self->childs.empty()) return -1;
            p.genericWaiting.wait(l);
        }
        Process *joined=self->zombies.front();
        self->zombies.pop_front();
        p.processes.erase(joined->pid);
        if(joined->waitCount!=0) errorHandler(UNEXPECTED);
        if(exit!=0) *exit=joined->exitCode;
        pid_t result=joined->pid;
        delete joined;
        return result;
    } else {
        //Wait on a specific child process
        map<pid_t,ProcessBase *>::iterator it=p.processes.find(pid);
        if(it==p.processes.end() || it->second->ppid!=self->pid
                || pid==self->pid) return -1;
        //Since the case when pid==0 has been singled out, this cast is safe
        Process *joined=static_cast<Process*>(it->second);
        if(joined->zombie==false)
        {
            //Process hasn't terminated yet
            if(options & WNOHANG) return 0;
            joined->waitCount++;
            joined->waiting.wait(l);
            joined->waitCount--;
            if(joined->waitCount<0 || joined->zombie==false)
                errorHandler(UNEXPECTED);
        }
        pid_t result=-1;
        if(joined->waitCount==0)
        {
            result=joined->pid;
            if(exit!=0) *exit=joined->exitCode;
            self->zombies.remove(joined);
            p.processes.erase(joined->pid);
            delete joined;
        }
        return result;
    }
}

Process::~Process() {}

Process::Process(const ElfProgram& program) : program(program), waitCount(0),
        zombie(false)
{
    //This is required so that bad_alloc can never be thrown when the first
    //thread of the process will be stored in this vector
    threads.reserve(1);
    //Done here so if not enough memory the new process is not even created
    image.load(program);
    unsigned int elfSize=program.getElfSize();
    unsigned int roundedSize=elfSize;
    if(elfSize<ProcessPool::blockSize) roundedSize=ProcessPool::blockSize;
    roundedSize=MPUConfiguration::roundSizeForMPU(roundedSize);
    //TODO: Till a flash file system that ensures proper alignment of the
    //programs loaded in flash is implemented, make the whole flash visible as
    //a big MPU region. This allows a program to read and execute parts of
    //other programs but not to write anything.
    extern unsigned char _elf_pool_start asm("_elf_pool_start");
    extern unsigned char _elf_pool_end asm("_elf_pool_end");
    unsigned int *start=reinterpret_cast<unsigned int*>(&_elf_pool_start);
    unsigned int *end=reinterpret_cast<unsigned int*>(&_elf_pool_end);
    unsigned int elfPoolSize=(end-start)*sizeof(int);
    elfPoolSize=MPUConfiguration::roundSizeForMPU(elfPoolSize);
    mpu=MPUConfiguration(start,elfPoolSize,
            image.getProcessBasePointer(),image.getProcessImageSize());
//    mpu=MPUConfiguration(program.getElfBase(),roundedSize,
//            image.getProcessBasePointer(),image.getProcessImageSize());
}

void *Process::start(void *argv)
{
    //This function is never called with a kernel thread, so the cast is safe
    Process *proc=static_cast<Process*>(Thread::getCurrentThread()->proc);
    if(proc==nullptr) errorHandler(UNEXPECTED);
    unsigned int entry=proc->program.getEntryPoint();
    Thread::setupUserspaceContext(entry,proc->image.getProcessBasePointer(),
        proc->image.getProcessImageSize());
    bool running=true;
    do {
        miosix_private::SyscallParameters sp=Thread::switchToUserspace();
        if(proc->fault.faultHappened())
        {
            running=false;
            proc->exitCode=SIGSEGV; //Segfault
            #ifdef WITH_ERRLOG
            iprintf("Process %d terminated due to a fault\n"
                    "* Code base address was 0x%x\n"
                    "* Data base address was %p\n",proc->pid,
                    proc->program.getElfBase(),
                    proc->image.getProcessBasePointer());
            proc->mpu.dumpConfiguration();
            proc->fault.print();
            #endif //WITH_ERRLOG
        } else running=proc->handleSvc(sp);
        if(Thread::testTerminate()) running=false;
    } while(running);
    {
        Processes& p=Processes::instance();
        Lock<Mutex> l(p.procMutex);
        proc->zombie=true;
        list<Process*>::iterator it;
        for(it=proc->childs.begin();it!=proc->childs.end();++it) (*it)->ppid=0;
        for(it=proc->zombies.begin();it!=proc->zombies.end();++it) (*it)->ppid=0;
        ProcessBase *kernel=p.processes[0];
        kernel->childs.splice(kernel->childs.begin(),proc->childs);
        kernel->zombies.splice(kernel->zombies.begin(),proc->zombies);
        
        map<pid_t,ProcessBase *>::iterator it2=p.processes.find(proc->ppid);
        if(it2==p.processes.end()) errorHandler(UNEXPECTED);
        it2->second->childs.remove(proc);
        if(proc->waitCount>0) proc->waiting.broadcast();
        else {
            it2->second->zombies.push_back(proc);
            p.genericWaiting.broadcast();
        }
    }
    return 0;
}

bool Process::handleSvc(miosix_private::SyscallParameters sp)
{
    try {
        switch(static_cast<Syscall>(sp.getSyscallId()))
        {
            case Syscall::OPEN:
            {
                auto name=reinterpret_cast<const char*>(sp.getParameter(0));
                int flags=sp.getParameter(1);
                if(mpu.withinForReading(name))
                {
                    int fd=fileTable.open(name,flags,
                        (flags & O_CREAT) ? sp.getParameter(2) : 0);
                    sp.setParameter(0,fd);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::CLOSE:
            {
                int result=fileTable.close(sp.getParameter(0));
                sp.setParameter(0,result);
                break;
            }

            case Syscall::READ:
            {
                int fd=sp.getParameter(0);
                void *ptr=reinterpret_cast<void*>(sp.getParameter(1));
                size_t size=sp.getParameter(2);
                if(mpu.withinForWriting(ptr,size))
                {
                    ssize_t result=fileTable.read(fd,ptr,size);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::WRITE:
            {
                int fd=sp.getParameter(0);
                void *ptr=reinterpret_cast<void*>(sp.getParameter(1));
                size_t size=sp.getParameter(2);
                if(mpu.withinForReading(ptr,size))
                {
                    ssize_t result=fileTable.write(fd,ptr,size);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::LSEEK:
            {
                off_t pos=sp.getParameter(2);
                pos|=static_cast<off_t>(sp.getParameter(1))<<32;
                off_t result=fileTable.lseek(sp.getParameter(0),pos,
                    sp.getParameter(3));
                sp.setParameter(0,result & 0xffffffff);
                sp.setParameter(1,result>>32);
                break;
            }

            case Syscall::STAT:
            {
                auto file=reinterpret_cast<const char*>(sp.getParameter(0));
                auto pstat=reinterpret_cast<struct stat*>(sp.getParameter(1));
                if(mpu.withinForReading(file) &&
                   mpu.withinForWriting(pstat,sizeof(struct stat)) && aligned(pstat))
                {
                    int result=fileTable.stat(file,pstat);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
            }

            case Syscall::LSTAT:
            {
                auto file=reinterpret_cast<const char*>(sp.getParameter(0));
                auto pstat=reinterpret_cast<struct stat*>(sp.getParameter(1));
                if(mpu.withinForReading(file) &&
                   mpu.withinForWriting(pstat,sizeof(struct stat)) && aligned(pstat))
                {
                    int result=fileTable.lstat(file,pstat);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
            }

            case Syscall::FSTAT:
            {
                auto pstat=reinterpret_cast<struct stat*>(sp.getParameter(1));
                if(mpu.withinForWriting(pstat,sizeof(struct stat)) && aligned(pstat))
                {
                    int result=fileTable.fstat(sp.getParameter(0),pstat);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::FCNTL:
            {
                //NOTE: some fcntl operations have an optional third parameter
                //which can be either missing, an int or a pointer.
                //Currenlty we do not support any with a pointer third arg, but
                //we arr on the side of safety and either pass the int or pass
                //zero. When we'll support those with the pointer, we'll
                //validate it here.
                int result,cmd=sp.getParameter(1);
                switch(cmd)
                {
                    case F_DUPFD: //Third parameter is int, no validation needed
                    case F_SETFD:
                    case F_SETFL:
                        result=fileTable.fcntl(sp.getParameter(0),cmd,
                                               sp.getParameter(2));
                    default:
                        result=fileTable.fcntl(sp.getParameter(0),cmd,0);
                }
                sp.setParameter(0,result);
                break;
            }

            case Syscall::IOCTL:
            {
                //TODO: need a way to validate ARG, for now we reject it
                //Not easy to move checks forward down filesystem code as that
                //code can also be called through kercalls from kthreads
                sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::ISATTY:
            {
                int result=fileTable.isatty(sp.getParameter(0));
                sp.setParameter(0,result);
                break;
            }

            case Syscall::GETCWD:
            {
                char *buf=reinterpret_cast<char*>(sp.getParameter(0));
                size_t size=sp.getParameter(1);
                if(mpu.withinForWriting(buf,size))
                {
                    int result=fileTable.getcwd(buf,size);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::CHDIR:
            {
                auto str=reinterpret_cast<const char*>(sp.getParameter(0));
                if(mpu.withinForReading(str))
                {
                    int result=fileTable.chdir(str);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::GETDENTS:
            {
                int fd=sp.getParameter(0);
                void *buf=reinterpret_cast<void*>(sp.getParameter(1));
                size_t size=sp.getParameter(2);
                if(mpu.withinForWriting(buf,size))
                {
                    int result=fileTable.getdents(fd,buf,size);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::MKDIR:
            {
                auto path=reinterpret_cast<const char*>(sp.getParameter(0));
                if(mpu.withinForReading(path))
                {
                    int result=fileTable.mkdir(path,sp.getParameter(1));
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::RMDIR:
            {
                auto str=reinterpret_cast<const char*>(sp.getParameter(0));
                if(mpu.withinForReading(str))
                {
                    int result=fileTable.rmdir(str);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::LINK:
            {
                sp.setParameter(0,-EMLINK); //Currently no fs supports hardlinks
                break;
            }

            case Syscall::UNLINK:
            {
                auto file=reinterpret_cast<const char*>(sp.getParameter(0));
                if(mpu.withinForReading(file))
                {
                    int result=fileTable.unlink(file);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::SYMLINK:
            {
//                 auto tgt=reinterpret_cast<const char*>(sp.getParameter(0));
//                 auto link=reinterpret_cast<const char*>(sp.getParameter(1));
//                 if(mpu.withinForReading(tgt) && mpu.withinForReading(link))
//                 {
//                     int result=fileTable.symlink(tgt,link);
//                     sp.setParameter(0,result);
//                 } else sp.setParameter(0,-EFAULT);
                sp.setParameter(0,-ENOENT); //Currently no writable fs supports symlinks
                break;
            }

            case Syscall::READLINK:
            {
                auto path=reinterpret_cast<const char*>(sp.getParameter(0));
                auto buf=reinterpret_cast<char*>(sp.getParameter(1));
                size_t size=sp.getParameter(2);
                if(mpu.withinForReading(path) &&
                   mpu.withinForWriting(buf,size))
                {
                    int result=fileTable.readlink(path,buf,size);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::RENAME:
            {
                auto oldp=reinterpret_cast<const char*>(sp.getParameter(0));
                auto newp=reinterpret_cast<const char*>(sp.getParameter(1));
                if(mpu.withinForReading(oldp) && mpu.withinForReading(newp))
                {
                    int result=fileTable.rename(oldp,newp);
                    sp.setParameter(0,result);
                } else sp.setParameter(0,-EFAULT);
                break;
            }

            case Syscall::CHMOD:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::FCHMOD:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::CHOWN:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::FCHOWN:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::LCHOWN:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::DUP:
            {
                int result=fileTable.dup(sp.getParameter(0));
                sp.setParameter(0,result);
                break;
            }

            case Syscall::DUP2:
            {
                int oldfd=sp.getParameter(0);
                int newfd=sp.getParameter(1);
                int result=fileTable.dup2(oldfd,newfd);
                sp.setParameter(0,result);
                break;
            }

            case Syscall::PIPE:
            {
                int fds[2];
                int result=fileTable.pipe(fds);
                sp.setParameter(1,result); //Does not overwrite r0 on purpose
                sp.setParameter(2,fds[0]);
                sp.setParameter(3,fds[1]);
                break;
            }

            case Syscall::ACCESS:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETTIME:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::SETTIME:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::NANOSLEEP:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETRES:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::ADJTIME:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::EXIT:
            {
                exitCode=(sp.getParameter(0) & 0xff)<<8;
                return false;
            }

            case Syscall::EXECVE:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::SPAWN:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::KILL:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::WAITPID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETPID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETPPID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETUID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETGID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETEUID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::GETEGID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::SETUID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::SETGID:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::MOUNT:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::UMOUNT:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            case Syscall::MKFS:
            {
                sp.setParameter(0,-EFAULT); //TODO: stub
                break;
            }

            default:
                exitCode=SIGSYS; //Bad syscall
                #ifdef WITH_ERRLOG
                iprintf("Unexpected syscall number %d\n",sp.getSyscallId());
                #endif //WITH_ERRLOG
                return false;
        }
    } catch(exception& e) {
        sp.setParameter(0,-ENOMEM);
    }
    return true;
}

pid_t Process::getNewPid()
{
    auto& p=Processes::instance();
    for(;;p.pidCounter++)
    {
        if(p.pidCounter<0) p.pidCounter=1;
        if(p.pidCounter==0) continue; //Zero is not a valid pid
        map<pid_t,ProcessBase*>::iterator it=p.processes.find(p.pidCounter);
        if(it!=p.processes.end()) continue; //Pid number already used
        return p.pidCounter++;
    }
}

} //namespace miosix

#endif //WITH_PROCESSES
