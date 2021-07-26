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

class connection_listener
{   
    sio::client &handler;
    std::condition_variable_any _cond;
    bool connect_finish = false;

    public:
        std::mutex _lock;

        /* Constructor. */
        connection_listener(sio::client& h);

        // connection_listener operator=(connection_listener obj)
        // {
        //     this->handler = obj.handler;
        //     this->_lock   = obj._lock;
        //     this->_cond   = obj._cond;
        //     this->connect_finish = obj.connect_finish;
        // }
        
        /* Function. */
        void on_connected();
        void on_close(sio::client::close_reason const& reason);
        void on_fail();
};

#endif