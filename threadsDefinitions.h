#include "uthreads.h"

typedef enum ThreadState {
    RUNNING,READY,BLOCKED
}ThreadState;

class Thread {
    private:
        int tid;
        ThreadState state;
        char stack[STACK_SIZE];
        int quantum;
        int quantum_sleep = 0;

    public:
        Thread():tid(0), state(RUNNING), quantum(1), quantum_sleep(0){};
        Thread(int id):tid(id), state(READY), quantum(0), quantum_sleep(0){};
        ThreadState get_state(){return state;}
        void set_state(ThreadState str){state = str;}
        int get_id(){return tid;}
        void set_quantum(int num){quantum+=num;}
        void set_quantum_sleep(int num){quantum_sleep=num;}
        void decreasing_quantum_sleeping(){quantum_sleep--;}
        int get_quantum_sleep(){return quantum_sleep;}
        int get_quantum(){return quantum;}
        char* get_stack(){return stack;}
};


