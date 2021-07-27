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

bool connection_listener::get_connect_finish()
{
    return connect_finish;
}

std::condition_variable_any* connection_listener::get_cond()
{
    return &_cond;
}