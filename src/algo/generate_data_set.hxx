# include "generate_data_set.hh"


template<class flight_controller_t,
	 class simulator_t>
Generator<flight_controller_t, simulator_t>::
Generator (std::vector<std::shared_ptr<flight_controller_t>> quads,
	   std::shared_ptr<simulator_t> sim_interface_):             
  quads_(std::move(quad)),
  sim_interface_(std::move(sim_interface))
{}


template<class flight_controller_t,
	 class simulator_t>
void Generator<flight_controller_t, simulator_t>::
run(){}
