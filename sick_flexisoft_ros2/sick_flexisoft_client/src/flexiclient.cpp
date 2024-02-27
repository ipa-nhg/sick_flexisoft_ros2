#include "../include/sick_flexisoft_client/flexiclient.h"
#include <cstdio>
#include <chrono>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace flexi;

FlexiClient::FlexiClient(input_handler_type input_handler): socket_(io_service_), reply_type_(FlexiHeader::UNDEFINED), reply_msg_(nullptr), input_handler_(input_handler) {}

bool FlexiClient::connect(const std::string& hostname, const std::string& port) {
    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), hostname, port);
    boost::asio::ip::tcp::resolver::iterator iterator = resolver.resolve(query);

    boost::system::error_code ec;
    socket_.connect(*iterator, ec);
    if (ec) return false;

    boost::asio::ip::tcp::no_delay no_delay(true);
    socket_.set_option(no_delay);

    return true;
}

void FlexiClient::start_worker() {
    if (workers_.size() == 0) trigger_async_read();
    workers_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}

void FlexiClient::run() {
    if (workers_.size() == 0) trigger_async_read();
    io_service_.run();
}

void FlexiClient::wait() {
    workers_.join_all();
}

void FlexiClient::close() {
    socket_.cancel();
}

void FlexiClient::trigger_async_read() {
    boost::asio::async_read(socket_,
        boost::asio::buffer(((uint8_t*)&msg_.payload) + msg_.length, msg_.missing_bytes()),
        boost::bind(&FlexiClient::handle_read,
            this,
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void FlexiClient::handle_read(const boost::system::error_code& ec, std::size_t bytes_transferred) {
    if (ec) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("FlexiClient"), "FlexiClient::handle_read(): received error receiving from socket => " << ec.message());
        close();
        return;
    }
    msg_.length += bytes_transferred;
    if (msg_.missing_bytes() == 0) {
        if (msg_.get_type() != FlexiHeader::UNDEFINED && reply_type_ == msg_.get_type()) {
            reply_type_ = FlexiHeader::UNDEFINED;
            if (reply_msg_) *reply_msg_ = msg_;
            reply_cond_.notify_one();
        }
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("FlexiClient"), "FlexiClient::handle_read: FlexiHeader: " << msg_.get_type());
        if (!msg_.is_error()) {
            if (msg_.get_type() == FlexiHeader::READ_DATA_REPLY || msg_.get_type() == FlexiHeader::UPDATE_DATA) {
                io_service_.post(boost::bind(input_handler_, msg_.payload.input));
            }
        } else {
            printf("ERROR 0x%04x\n", (uint16_t)msg_.get_type());
        }
        msg_.length = 0;
    }
    trigger_async_read();
}

bool FlexiClient::send(const FlexiMsg& msg, FlexiMsg* reply) {
    if (msg.missing_bytes() > 0) return false;

    boost::mutex::scoped_lock send_lock(write_mutex_);
    // boost::unique_lock<boost::mutex> wait_lock(wait_mutex_);
    std::unique_lock<std::mutex> wait_lock(wait_mutex_);

    boost::system::error_code ec;
    boost::asio::write(socket_, boost::asio::buffer(&msg.payload, msg.length), boost::asio::transfer_all(), ec);

    if (ec) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("FlexiClient"), "FlexiClient::send(): received error sending to socket => " << ec.message());
        close();
        return false;
    }

    reply_msg_ = reply;
    reply_type_ = msg.get_reply_type();
    while (reply_type_ != FlexiHeader::UNDEFINED) {
        RCLCPP_DEBUG(rclcpp::get_logger("FlexiClient"), "FlexiClient::send(): waiting for reply_cond");
        if (reply_cond_.wait_for(wait_lock, std::chrono::milliseconds(5000))== std::cv_status::timeout) {
            RCLCPP_DEBUG(rclcpp::get_logger("FlexiClient"), "FlexiClient::send(): reply_cond timedout");
            close();
            return false;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("FlexiClient"), "FlexiClient::send(): reply_cond fullfilled");
    }
    reply_msg_ = nullptr;

    return true;
}
