//
// Created by vis75817 on 1/11/2022.
//

#ifndef MCU2_CHAINABLEOBSERVER_H
#define MCU2_CHAINABLEOBSERVER_H
#include <array>
#include <functional>

class ChainableObserver {
    void then(ChainableObserver *next);
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
    // implement comparable operators
};


class Scheduler{
private:
    std::array<Task, 10> tasks;
    StaticBuffer<Task, 10> ready_tasks;
    int time = 0;
public:
    void update(){
        // iterate through sleeping tasks
        for(auto task: tasks){
            if(task.time_start <= time){
                ready_tasks.enqueue(task);
            }
        }
    }
    void run(){
        while(!ready_tasks.isEmpty()){
            const auto task = ready_tasks.dequeue();
            // todo: find a clean way to do this without std::function since newlib seems to have issues with it
            task.func();
        }
    }
};
#endif //MCU2_CHAINABLEOBSERVER_H
