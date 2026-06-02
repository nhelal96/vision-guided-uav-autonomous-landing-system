#include "qr_landing/algorithm.h"
#include "qr_landing/helper.h"

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <string.h>
#include <vector>
#include <map>

#include<iostream>

namespace QRLanding{

  Algorithm::Algorithm(){
      // Configure scanner
      scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
      scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  }


  std::vector<std::string> Algorithm::prepare_string(std::string s){
      std::vector<std::string> results;
      std::stringstream  ss(s);
      std::string str;
      while (ss.good()) {
          getline(ss, str, ',');
          results.push_back(str);
      }
      return results;
  }

  std::map<std::string,std::string> Algorithm::make_dict
                        (std::vector<std::string>&tokens){

      std::map<std::string,std::string>data;
      for(auto token:tokens){
          std::stringstream ss(token);
          std::string str;
          std::vector<std::string>res;

          while(ss.good()){
              getline(ss,str,':');
              res.push_back(str);
          }
          data.insert(std::pair<std::string,std::string>(res[0],res[1]));
      }
      return data;

  }

  bool Algorithm::decode(cv::Mat&img){

      cv::Mat gray;

      cv::cvtColor(img,gray,CV_BGR2GRAY); // for opencv v2.*
      //cv::cvtColor(img,gray,cv::COLOR_BGR2GRAY); //for opencv v3.*

      zbar::Image image(img.cols,img.rows,"Y800",(uchar*)gray.data,gray.cols*gray.rows);

      int n=scanner.scan(image);
      if(!n)return false;

      for(zbar::Image::SymbolIterator symbol=image.symbol_begin(); symbol!=image.symbol_end(); ++symbol){
        QRObject obj;
        obj.type=symbol->get_type();
        std::string data=symbol->get_data();

        auto res=prepare_string(data);
        obj.dict_data=make_dict(res);

        for(int i=0; i<symbol->get_location_size(); ++i)
            obj.location.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));


        objects.push_back(obj);
      }

    return true;
  }

  int Algorithm::check(){

      for(int i=0; i<objects.size(); ++i){
          bool c=1;

          auto obj=objects[i];
          c&=obj.type==64; // QR-Code

          auto dict=obj.dict_data;
          c&=dict["name"]=="isoft";
          c&=dict["id"]=="1";
          c&=dict["land"]=="true";

          if(c)return i;
      }

      return -1;
  }

  void Algorithm::project(int indx,ResultPoint&res_p){

      std::vector<cv::Point>points=objects[indx].location;
      res_p.p0[0]=points[0].x;
      res_p.p0[1]=points[0].y;
      res_p.p2[0]=points[2].x;
      res_p.p2[1]=points[2].y;
  }


  bool Algorithm::process(cv::Mat&img, ResultPoint&res_p){
      // clear data
      objects.clear();

      bool ret=decode(img);
      if(!ret)return ret;

      int ret_i=check();
      if(ret_i==-1)return false;

      project(ret_i,res_p);
      return true;
  }
}
