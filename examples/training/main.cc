/**
 * @file main.cc
 * @brief train the dataset
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 *
 */

/*  C++ Standard library include */
# include <chrono>
# include <cstdlib>
# include <string>
# include <thread>
# include <vector>

/*  locale defined include */
# include "training.hh"

/*
 *  Main file: Start generate dataset
 */
int main(int argc, char* argv[])
{

  /*  Init logging system */
  Log log;
  log.init();
      
  LogInfo() << "Start training...";
  Train trainer;    
  trainer.load_data_set(settings.dataset());
  trainer.run(settings);
  LogInfo() << "Finished training...";  
}
