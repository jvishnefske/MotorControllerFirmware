//
// Created by vis75817 on 1/11/2022.
//

#ifndef MCU2_CHAINABLEOBSERVER_H
#define MCU2_CHAINABLEOBSERVER_H


class ChainableObserver {

};


template<class T, int size>
class StaticBuffer{
private:
    std::array<T, size> buffer;
    int head = 0;
    int tail = 0;
public:
    void enqueue(T item){
        buffer[head] = item;
        head = (head + 1) % size;
        if(head == tail){
            tail = (tail + 1) % size;
        }
    }
    T dequeue(){
        T item = buffer[tail];
        tail = (tail + 1) % size;
        return item;
    }
    bool isEmpty(){
        return head == tail;
    }
};

struct Task{
    std::function<void()> func;
    int time_start;
    int time_period;
};

class Scheduler{
private:
    std::array<Task, 10> tasks;
    StaticBuffer<Task, 10> ready_tasks;
    int time = 0;
public:
    void update(){
        // iterate through sleeping tasks
        while(!sleeping_tasks.isEmpty()){
            Task task = sleeping_tasks.dequeue();
            if(task.time_start <= time){
                ready_tasks.enqueue(task);
            }
            else{
                sleeping_tasks.enqueue(task);
                break;
            }
        }
    }
    void run(){
        while(!ready_tasks.isEmpty()){
            const Task task = ready_tasks.dequeue();
            task.func();
        }
    }
};
#endif //MCU2_CHAINABLEOBSERVER_H
