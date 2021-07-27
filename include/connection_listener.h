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

#ifndef CONNECTION_LISTERNER_H
#define CONNECTION_LISTERNER_H

// std::mutex _lock;

class connection_listener
{   
    private:
        sio::client &_handler;

    public:
        /* Constructor. */
        connection_listener(sio::client& h): _handler(h) {}

        /* Function. */
        void on_connected();
        void on_close(sio::client::close_reason const& reason);
        void on_fail();
        
};

#endif