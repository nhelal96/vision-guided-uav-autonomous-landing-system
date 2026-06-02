/**
  * algorithm.h
  * by:       Nour Helal
  * for:      Unix amd64
  * purpose:  defining the algorithms of finding QR code
**/


#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "qr_landing/helper.h"

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <string.h>
#include <vector>



namespace QRLanding{

    class Algorithm{

    zbar::ImageScanner scanner;
    std::vector<QRObject>objects;

    public:
        Algorithm();

        /**
          * @brief        main function of the class
          * @param[in]    img, an input imgage with QR code in it
          * @param[out]   res_p, holder of the position of QR in the image
          * @return       true if there is an QR in the image, false otherwise
        **/
        bool process(cv::Mat&, ResultPoint&);

        /**
          * @brief        maps the coordinates of the QR in image
                          to local coordinates matches controller
                          coordinate system
          * @param[in]    indx, index of the solution in the objects
                          vector
          * @param[out]   res_p, holder of the position of solution
        **/
        void project(int ,ResultPoint&);

        /**
          * @brief        searches for QR in the set of solutions
                          and checks if  QR contains the coorect data
          * @return       index of the solution, -1 if there is no solution
        **/
        int check();

        /**
          * @brief        finds all QR in an image and thier data
          * @param[in]    img, image to search for QR in
          * @return       true if there is a solution, fasle otherwise
        **/
        bool decode(cv::Mat&);

        /**
          * @brief        make a dictionary of QR data
          * @param[in]    tokens, vector of strings contains QR data
          * @return         map of QR data in structured form
        **/
        std::map<std::string,std::string> make_dict(std::vector<std::string>&);

        /**
          * @bief         prepare QR data to be easy to search in
          * @param[in]    string contains QR data
          * @return       vector contains QR data seperated
        **/
        std::vector<std::string> prepare_string(std::string );
    };
}


#endif // ALGORITHM_H
