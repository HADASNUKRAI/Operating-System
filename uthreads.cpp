#include "uthreads.h"
#include "threadsDefinitions.h"
#include <cstdlib>
#include <sys/time.h>
#include <iostream>
#include <vector>
#include <queue>
#include <csetjmp>
#include <unistd.h>
#include <map>
#include <signal.h>
#include <set>
#include <algorithm>


#define JB_SP 6
#define JB_PC 7
#define ERROR -1
#define SUCCESS 0
#define ERROR_SYSTEM_MSG_PREFIX "system error: "
#define ERROR_THREAD_MSG_PREFIX "thread library error: "
#define QUANTOM_ERROR_MSG "Quantum is negative"
#define SIG_CALL_ERROR_MSG "Failed to operate the sigaction function"
#define SIGEMPTYSET_ERROR_MSG "Failed to operate the sigemptyset function"
#define TIMER_ERROR_MSG "Failed to operate timer"
#define ENTRY_ERROR_MSG "Entry point is null"
#define MAXTHREADS_ERROR_MSG "Can't add more threads due to max threads"
#define NO_THREAD_ERROR_MSG "This Thread ID doesnt exist"
#define MAIN_THREAD_BLOCK_ERROR_MSG "Thread id 0, cannot be blocked"



typedef unsigned long address_t;

Thread* current_thread;
std::map<int, Thread*> threads;
std::deque<Thread*>ready_threads;
std::deque<Thread*>block_threads;
std::deque<Thread*>sleeping_threads;
static std::vector<int>valid_id;
static std::vector<int>deleted_id;
sigset_t signal_set;//set of signals
sigjmp_buf saved_env[MAX_THREAD_NUM];
int total_quantums=0;
struct sigaction sa;
struct itimerval timer;

void delete_from_queue(int tid);
bool is_thread_exist(int tid);
int findMinimumId();
int findMaxId();
int uthread_get_tid();


/**
 * Initializes and sets a virtual timer that generates a SIGVTALRM
 * each interval time .
 * @param quantum_usecs length of a quantum in micro-seconds.
 * @return failed if the setittimer function failed otherwise success
 */
int init_timer(int quantum_usecs = timer.it_value.tv_usec) {

    timer.it_value.tv_sec = quantum_usecs/1000000;//in seconds
    timer.it_value.tv_usec = quantum_usecs%1000000;//in microseconds
    timer.it_interval.tv_sec = quantum_usecs/1000000;
    timer.it_interval.tv_usec = quantum_usecs%1000000;
    //update the general timer
    if (setitimer(ITIMER_VIRTUAL, &timer, NULL)<0){
        std::cerr<<ERROR_SYSTEM_MSG_PREFIX<<TIMER_ERROR_MSG<<std::endl;
        return ERROR;
    }
    return SUCCESS;
}

/**
 * Update the current thread after switching context
 */
void update_current_thread(){
    total_quantums++;
    current_thread = ready_threads.front();
    ready_threads.pop_front();
    current_thread->set_state(RUNNING);
    current_thread->set_quantum(1);
}




/**
 * Checking up if the quantum thread of each sleeping thread had finish
 */
void threads_wakes_up(){
    for (Thread* thread:sleeping_threads){
        thread->decreasing_quantum_sleeping();
        if (thread->get_quantum_sleep()==0){
            delete_from_queue(thread->get_id());
            thread->set_state(READY);
            ready_threads.push_back(thread);
        }
    }
}





/**
 * Handling with signals
 * @param sig_tid the id of the signal
 */
void time_handler(int sig_tid){
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);//block all the other signals
    threads_wakes_up();
    int id = uthread_get_tid();
    current_thread->set_state(READY);
    //removing the thread to the ready queue because his quantum has expired
    ready_threads.push_back(current_thread);
    update_current_thread();
    //save the environment of the current signal after he got a signal
    if (sigsetjmp(saved_env[id], 1) == 0){
        siglongjmp(saved_env[current_thread->get_id()], 1);
    }
    sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);//unblock all the other signals
}


/**
 * Handling with signals
 * @param quantum_usecs length of a quantum in micro-seconds.
 * @retuen the function for handling the signals
 */
int init_signal_handler(int quantum_usecs) {
    sa = {nullptr};
    sa.sa_handler = &time_handler;
    //checking if he was succeeded dealing with the signal
    if (sigaction(SIGVTALRM, &sa, NULL) < 0) {
        std::cerr<<ERROR_SYSTEM_MSG_PREFIX<<SIG_CALL_ERROR_MSG<<std::endl;
        return ERROR;
    }
    return init_timer(quantum_usecs);
}



/**
 * @brief initializes the thread library.
 *
 * Once this function returns, the main thread (tid == 0) will be set as RUNNING. There is no need to
 * provide an entry_point or to create a stack for the main thread - it will be using the "regular" stack and PC.
 * @param quantum_usecsthe length of a quantum in micro-seconds.
 * @return On success, return 0. On failure, return -1.
*/
int uthread_init(int quantum_usecs){

    if(quantum_usecs<=0){
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<QUANTOM_ERROR_MSG<<std::endl;
        return ERROR;
    }

    total_quantums = 1;

    if (init_signal_handler(quantum_usecs)==ERROR){
        std::cerr<<ERROR_SYSTEM_MSG_PREFIX<<SIG_CALL_ERROR_MSG<<std::endl;
        return ERROR;
    }


    Thread* thread = new Thread();
    valid_id.push_back(0);
    threads[0] = thread;
    current_thread = thread;
    sigemptyset(&signal_set); //init the signal set
    sigaddset(&signal_set, SIGVTALRM);
    return SUCCESS;
}


/**
 * A translation is required when using an address of a variable.
   Use this as a black box in your code.
*/
address_t translate_address(address_t addr){
    address_t ret;
    asm volatile("xor    %%fs:0x30,%0\n"
                 "rol    $0x11,%0\n"
        : "=g" (ret)
        : "0" (addr));
    return ret;
}



/**
 * function sets up the initial state of a thread, including its stack pointer (SP) and program counter (PC),
 * so that when the thread is resumed using siglongjmp, it will start executing from
 * the specified entry point with the correct stack setup.
 * @param thread
 * @param entry_point the function to invoke the thread
 * @return 0 at success, -1 at failure
 */
int setup_thread(Thread* thread, thread_entry_point entry_point) {
    address_t sp = (address_t) thread->get_stack() + STACK_SIZE - sizeof(address_t);
    address_t pc = (address_t) entry_point;
    sigsetjmp(saved_env[thread->get_id()], 1);
    (saved_env[thread->get_id()]->__jmpbuf)[JB_SP] = translate_address(sp);
    (saved_env[thread->get_id()]->__jmpbuf)[JB_PC] = translate_address(pc);

    if (sigemptyset(&saved_env[thread->get_id()]->__saved_mask) == -1) {
        std::cerr<<ERROR_SYSTEM_MSG_PREFIX<<SIGEMPTYSET_ERROR_MSG<<std::endl;
        return ERROR;
    }
    return SUCCESS;
}


/**
 * The thread is added to the end of the READY threads list.
 * The uthread_spawn function should fail if it would cause the number of concurrent threads to exceed the
 * limit (MAX_THREAD_NUM).
 * @param entry_point the function for invoke the thread
 * @return On success, return the ID of the created thread. On failure, return -1.
 */
int uthread_spawn(thread_entry_point entry_point) {

    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
    if (entry_point == nullptr) {
        std::cerr<<ERROR_SYSTEM_MSG_PREFIX<<ENTRY_ERROR_MSG<<std::endl;
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return ERROR;
    }

    if (findMaxId() >= MAX_THREAD_NUM) {
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<MAXTHREADS_ERROR_MSG<<std::endl;
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return ERROR;
    }

    int num = findMaxId();
    if (deleted_id.size() != 0){
        num = findMinimumId();
        auto it = std::find(deleted_id.begin(), deleted_id.end(), num);

        if(it != deleted_id.end()){
            deleted_id.erase(it);
        }
    }

    valid_id.push_back(num);
    Thread *thread = new Thread(num);
    ready_threads.push_back(thread);
    threads[num] = thread;
    //allocate the stack for the thread
    if (setup_thread(thread, entry_point) == 0) {
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return thread->get_id();
    }

    sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
    return ERROR;
}




/**
 * @brief Terminates the thread with ID tid and deletes it from all relevant control structures.
 *
 * All the resources allocated by the library for this thread should be released. If no thread with ID tid exists it
 * is considered an error. Terminating the main thread (tid == 0) will result in the termination of the entire
 * process using exit(0) (after releasing the assigned library memory).
 *
 * @return The function returns 0 if the thread was successfully terminated and -1 otherwise. If a thread terminates
 * itself or the main thread is terminated, the function does not return.
*/
int uthread_terminate(int tid) {

    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
    if (!is_thread_exist(tid)) {
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<NO_THREAD_ERROR_MSG<<std::endl;
        return ERROR;
    }
    if (tid == 0){
        for (auto it = threads.begin(); it != threads.end(); ++it) {
            delete it->second;  // Delete the Thread* object
        }
        // Clear the map
        threads.clear();
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        exit(0);
    }
    deleted_id.push_back(tid);

    auto it = std::find(valid_id.begin(), valid_id.end(), tid);
    if(it != valid_id.end()){
        valid_id.erase(it);
    }

    delete_from_queue(tid);

    //delete from all the threads
    auto thread = threads.find(tid); // Find the element by key
    if (thread != threads.end()) {
        delete thread->second; // Delete the Thread* object
        threads.erase(thread); // Erase the element from the map
    }

    sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);

    return SUCCESS;
}


/**
 * Finding the minimum id in the deleted ids vector
 * @return the minimum id
 */
int findMinimumId() {
    int min = MAX_THREAD_NUM;
    if(deleted_id.size()>0) {
        for (long unsigned int i = 0; i < deleted_id.size(); i++) {
            if (deleted_id[i] < min) {
                min = deleted_id[i];
            }
        }
    }
    return min;
}


/**
 * Finding the maximum id in the used ids and add 1 for using it as the id for the new created thread
 * @return the id maximum id+1
 */
int findMaxId() {
    int max = -1;
    for (long unsigned int i = 0; i < valid_id.size(); i++) {
        if (valid_id[i] > max) {
            max = valid_id[i];
        }
    }
    return max+1;
}


/**
 * Delete from queue
 * @param tid the id of specific thread
 */
void delete_from_queue(int tid) {
    if (threads[tid]->get_state() == RUNNING){
        update_current_thread();
        init_timer();
        if (sigsetjmp(saved_env[tid], 1)==0){
            siglongjmp(saved_env[current_thread->get_id()], 1);
        }
        return;
    }

    std::deque<Thread*>new_queue;

    if (threads[tid]->get_state() == READY) {
        for (Thread* thread:ready_threads){
            if (thread->get_id()!=tid){
                new_queue.push_front(thread);
            }
        }
        ready_threads.swap(new_queue);
    }
    else if (threads[tid]->get_state() == BLOCKED) {
        for (Thread* thread:block_threads){
            if (thread->get_id()!=tid){
                new_queue.push_front(thread);
            }
        }
        block_threads.swap(new_queue);
    }
    else{//delete from sleeping queue
        for (Thread* thread:sleeping_threads){
            if (thread->get_id()!=tid){
                new_queue.push_front(thread);
            }
        }
        sleeping_threads.swap(new_queue);
    }
}



/**
 * @brief Blocks the thread with ID tid. The thread may be resumed later using uthread_resume.
 *
 * If no thread with ID tid exists it is considered as an error. In addition, it is an error to try blocking the
 * main thread (tid == 0). If a thread blocks itself, a scheduling decision should be made. Blocking a thread in
 * BLOCKED state has no effect and is not considered an error.
 *
 * @return On success, return 0. On failure, return -1.
*/
int uthread_block(int tid){
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
    if(!is_thread_exist(tid)){
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<NO_THREAD_ERROR_MSG<<std::endl;
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return ERROR;
    }

    if (tid == 0){
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<MAIN_THREAD_BLOCK_ERROR_MSG<<std::endl;
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return ERROR;
    }

    if (threads[tid]->get_state() != BLOCKED){
        delete_from_queue(tid);
    }
    else{
        return SUCCESS;
    }

    threads[tid]->set_state(BLOCKED);
    block_threads.push_back(threads[tid]);
    sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
    return SUCCESS;
}


/**
 * @brief Resumes a blocked thread with ID tid and moves it to the READY state.
 *
 * Resuming a thread in a RUNNING or READY state has no effect and is not considered as an error. If no thread with
 * ID tid exists it is considered an error.
 *
 * @return On success, return 0. On failure, return -1.
*/
int uthread_resume(int tid){
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);

    if(!is_thread_exist(tid)){
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<NO_THREAD_ERROR_MSG<<std::endl;
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return ERROR;
    }

    if (threads[tid]->get_state() == BLOCKED){
        delete_from_queue(tid);
    }

    threads[tid]->set_state(READY);
    ready_threads.push_back(threads[tid]);
    sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
    return SUCCESS;
}



/**
 * @brief Blocks the RUNNING thread for num_quantums quantums.
 *
 * Immediately after the RUNNING thread transitions to the BLOCKED state a scheduling decision should be made.
 * After the sleeping time is over, the thread should go back to the end of the READY queue.
 * If the thread which was just RUNNING should also be added to the READY queue, or if multiple threads wake up
 * at the same time, the order in which they're added to the end of the READY queue doesn't matter.
 * The number of quantums refers to the number of times a new quantum starts, regardless of the reason. Specifically,
 * the quantum of the thread which has made the call to uthread_sleep isn’t counted.
 * It is considered an error if the main thread (tid == 0) calls this function.
 *
 * @return On success, return 0. On failure, return -1.
*/
int uthread_sleep(int num_quantums){
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
    if (uthread_get_tid() == 0){
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<MAIN_THREAD_BLOCK_ERROR_MSG<<std::endl;
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return ERROR;
    }

    int current_id = uthread_get_tid();
    threads[current_id]->set_quantum_sleep(num_quantums);
    threads[current_id]->set_state(BLOCKED);
    sleeping_threads.push_back(threads[current_id]);
    update_current_thread();
    init_timer();

    if (sigsetjmp(saved_env[current_id], 1) == 0){
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        siglongjmp(saved_env[current_thread->get_id()], 1);
    }

    sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
    return SUCCESS;
}



/**
 * @brief Returns the thread ID of the calling thread.
 *
 * @return The ID of the calling thread.
*/
int uthread_get_tid(){
    return current_thread->get_id();
}



/**
 * @return The total number of quantums.
*/
int uthread_get_total_quantums(){
    return total_quantums;
}


/**
 * @brief Returns the number of quantums the thread with ID tid was in RUNNING state.
 *
 * On the first time a thread runs, the function should return 1. Every additional quantum that the thread starts should
 * increase this value by 1 (so if the thread with ID tid is in RUNNING state when this function is called, include
 * also the current quantum). If no thread with ID tid exists it is considered an error.
 *
 * @return On success, return the number of quantums of the thread with ID tid. On failure, return -1.
*/
int uthread_get_quantums(int tid){
    if(!is_thread_exist(tid)){
        std::cerr<<ERROR_THREAD_MSG_PREFIX<<NO_THREAD_ERROR_MSG<<std::endl;
        sigprocmask(SIG_UNBLOCK, &signal_set, nullptr);
        return ERROR;
    }
    return threads[tid]->get_quantum();
}


/**
 * Checking if the given id is already exist
 * @param tid the given id
 * @return true if the id is exist and otherwisw false
 */
bool is_thread_exist(int tid) {
    for (int val:valid_id){
        if (val == tid){
            return true;
        }
    }

    return false;
}







