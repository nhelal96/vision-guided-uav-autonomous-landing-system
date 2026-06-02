/**
  * Algorithm.h
  * by:       Nour Helal
  * for:      Unix amd64
  * purpose:  defines the algorithm to find good take off area
**/


#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "safe_takeoff/helper.h"

#include <vector>

namespace SafeTakeoff{

  class Algorithm{
      const double MAX_ALT=1;
      const double MIN_ALT=0.1;
      int measure[YS][XS];
      int dp[YS][XS];
      int max_i,max_j,max_of_s;

    public:
      Algorithm(){};

      /**
        * @brief        contructs the searching mesh area
        * @param[in]    mesh, 2D vector contains distance data
      **/
      void construct(std::vector<std::vector<double> >&);

      /**
        * @brief        searches for a solution in mesh data
        * @param[in]    mesh, 2D vector contains distance data
        * @param[out]   res_p, holder for solution position
        * @return       true if there is a solution, false otherwise
      **/
      bool search(std::vector<std::vector<double> >&,ResultPoint&);

      /**
        * @brief        check if the area above the drone is free or not
      **/
      bool search_rect();

      /**
        * @brief        contruct mesh data and checks for free area
                        exactly above the drone
        * @return       true if the area above the drone is clear,
                        false otherwise
      **/
      bool is_clear_area(std::vector<std::vector<double> >&);

      /**
        * @brief        main function of the class
      **/
      bool process(std::vector<std::vector<double> >&,ResultPoint&);
  };


}


#endif // ALGORITHM_H
