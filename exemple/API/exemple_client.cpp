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
// #include <ctime>
#include <cstdlib>

#define HIGHLIGHT(__O__) std::cout<<"\e[1;31m"<<__O__<<"\e[0m"<<std::endl
#define EM(__O__) std::cout<<"\e[1;30;1m"<<__O__<<"\e[0m"<<std::endl


using namespace sio;
using namespace std;
std::mutex _lock;

std::condition_variable_any _cond;
bool connect_finish = false;

string map_number   = "map_1";
string localisation = "DVIC";
string name         = "MK2R2_1";

class connection_listener
{
    sio::client &handler;

public:
    
    connection_listener(sio::client& h):
    handler(h)
    {
    }
    

    void on_connected()
    {
        _lock.lock();
        _cond.notify_all();
        connect_finish = true;
        _lock.unlock();
    }
    void on_close(client::close_reason const& reason)
    {
        std::cout<<"sio closed "<<std::endl;
        exit(0);
    }   
    void on_fail()
    {
        std::cout<<"sio failed "<<std::endl;
        exit(0);
    }
};

socket::ptr current_socket;
void bind_events()
{
    current_socket->on("good", sio::socket::event_listener_aux([&](string const& name, message::ptr const& data, bool isAck,message::list &ack_resp)
                       {
                           _lock.lock();
                std::cout << "I have the good map ! What would you expect ?" << std::endl;
                // cout << data->get_string() << endl;
                          _lock.unlock();
                       }));


    current_socket->on("download", sio::socket::event_listener_aux([&](string const& name, message::ptr const& data, bool isAck,message::list &ack_resp)
                       {
                           _lock.lock();
                cout << name << endl;
                cout << data->get_string() << endl;
                          _lock.unlock();

            // WGET FUNCTION TO IMPORT !
                       }));

    current_socket->on("command_to_do", sio::socket::event_listener_aux([&](string const& name, message::ptr const& data, bool isAck,message::list &ack_resp)
                       {
                           _lock.lock();
                cout << name << endl;
                cout << data->get_string() << endl;
                          _lock.unlock();
                       }));

    current_socket->on("position_to_reach", sio::socket::event_listener_aux([&](string const& name, message::ptr const& data, bool isAck,message::list &ack_resp)
                       {
                           _lock.lock();
                cout << name << endl;
                cout << data->get_string() << endl;
                          _lock.unlock();
                       }));
}

void check_map()
{
    string message = map_number + "/" + localisation; 
    
    current_socket->emit("check_map", message);
}

void position()
{
    // int event_name = 1;

    sio::message::ptr arr = array_message::create();

    arr->get_vector().push_back(string_message::create(localisation));
    arr->get_vector().push_back(int_message::create(12));
    arr->get_vector().push_back(int_message::create(23));


    current_socket->emit("position", arr);
}

int main(int argc ,const char* args[])
{

    sio::client h;
    connection_listener l(h);
    
    h.set_open_listener(std::bind(&connection_listener::on_connected, &l));
    h.set_close_listener(std::bind(&connection_listener::on_close, &l,std::placeholders::_1));
    h.set_fail_listener(std::bind(&connection_listener::on_fail, &l));
    h.connect("http://127.0.0.1:5000");
    
    _lock.lock();
    if(!connect_finish)
    {
        cout << "wait\n";
            _cond.wait(_lock);
    }
    _lock.unlock();

    current_socket = h.socket();
    
    // Give its name so that the API knows who it is talking to 
    current_socket->emit("name", name);

    while(true)
    {
        check_map();
        bind_events();
        position();
        sleep(2);
    }

    return 0;
}