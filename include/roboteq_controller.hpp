#pragma once

#include <memory> 

#include "i_comm.hpp"

namespace roboteq {

    class roboteq_controller {

        public :
        explicit roboteq_controller(std::unique_ptr<i_comm> &&comm);
        roboteq_controller & operator= ( const roboteq_controller& ) = delete;
        ~roboteq_controller(){}

        bool download(runtime_command command, uint8_t subindex, uint32_t data);
        uint32_t upload(runtime_query query, uint8_t subindex);

        private :
        std::unique_ptr<i_comm> _comm;

    };

} //namespace roboteq 

