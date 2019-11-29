
namespace roboteq {

void roboteq_controller::roboteq_controller(std::unique_ptr<i_comm> &&comm) : _comm(comm){

}

void roboteq_controller::SetParameters(int16_t _velocity, long _accel, long _decel, uint8_t _channel) {

    _comm->SetParameters(_velocity, _accel, _decel, _channel);
}

} // namespace roboteq