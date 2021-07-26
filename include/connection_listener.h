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

class connection_listener
{
    sio::client &handler;
    std::mutex _lock;
    std::condition_variable_any _cond;
    bool connect_finish = false;

    public:
        
        connection_listener(sio::client& h);
        connection_listener();

        void on_connected();
        void on_close(sio::client::close_reason const& reason);
        void on_fail();
};