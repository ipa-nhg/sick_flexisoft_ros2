#ifndef H_FLEXI_CLIENT
#define H_FLEXI_CLIENT

#include "flexiproto.h"

#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>

namespace flexi {

class FlexiClient {
public:
    bool connect(const std::string& hostname, const std::string& port);

    bool send(const FlexiMsg& msg, FlexiMsg* reply = nullptr);

    void start_worker();
    void run();
    void wait();

    void close();

    using input_handler_type = boost::function<void(const FlexiInputData&)>;
    FlexiClient(input_handler_type input_handler);

protected:
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;

    FlexiMsg msg_;
    boost::mutex write_mutex_;

    std::mutex wait_mutex_;
    volatile FlexiHeader::CommandType reply_type_;
    std::condition_variable reply_cond_;
    FlexiMsg* reply_msg_;

    input_handler_type input_handler_;
    boost::thread_group workers_;

    bool write_msg(const FlexiMsg& msg);
    void trigger_async_read();
    void handle_read(const boost::system::error_code& ec, std::size_t bytes_transferred);
};

};

#endif
