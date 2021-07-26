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

connection_listener::connection_listener(sio::client& h):
handler(h)
{
}

connection_listener::connection_listener()
{}

void connection_listener::on_connected()
{
    _lock.lock();
    _cond.notify_all();
    connect_finish = true;
    _lock.unlock();
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