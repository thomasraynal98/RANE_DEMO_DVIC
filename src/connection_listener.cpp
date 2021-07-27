#include <sio_client.h>
#include <unistd.h>
#include <functional>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <map>
#include <vector>
#include <list>
#include <cstdlib>

#include "../include/connection_listener.h"

bool connect_finish;
std::condition_variable_any _cond;

void connection_listener::on_connected()
{
    // _lock.lock();
    _cond.notify_all();
    connect_finish = true;
    // _lock.unlock();
}

void connection_listener::on_close(sio::client::close_reason const& reason)
{
    std::cout<<"sio closed "<<std::endl;
    exit(0);
}  

void connection_listener::on_fail()
{
    std::cout<<"sio failed "<<std::endl;
    exit(0);
}