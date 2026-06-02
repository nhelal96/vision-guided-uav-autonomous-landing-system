#include "SafeLanding/algorithm.h"
#include "SafeLanding/helper.h"

#include "math.h"

#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include<iostream>
#include<fstream>
#include<math.h>
#include <string>
#include<sstream>
#include<vector>

namespace SafeLanding{


  Algorithm::Algorithm(){
      depthFOVx=86.0*deg;
      depthFOVy=57.0*deg;
      depthx0=0.5;
      depthy0=0.5;
      depthz0=0.8;
      depthz1=1.;
      error=depthz1*0.99;
      depthcfg=_DepthImage_invert_z|_DepthImage_cos_correct_z; // if the closer objects are darker, do not invert
      cloud=std::vector<std::vector<DepthPoint> >(YS,std::vector<DepthPoint>(XS));
  }


  void Algorithm::project(ResultPoint&res){
      res.valid=true;

      double delta=0;
      double x=atan2(p0[0],p0[2])/depthFOVx;
      double y=atan2(p0[1],p0[2])/depthFOVy;
      if(int(depthcfg&_DepthImage_cos_correct_z))
          y*=(YS*depthFOVx)/(depthFOVy*XS);

      x=(x+0.5)*XS;
      y=(y+0.5)*YS;
      if(y<0){
        delta=0-y;
        y+=delta;
      }
      res.p0[0]=x;
      res.p0[1]=y;

      x=atan2(p2[0],p2[2])/depthFOVx;
      y=atan2(p2[1],p2[2])/depthFOVy;
      if(int(depthcfg&_DepthImage_cos_correct_z))
          y*=(YS*depthFOVx)/(depthFOVy*XS);

      x=(x+0.5)*XS;
      y=(y+0.5)*YS;
      y+=delta;
      res.p2[0]=x;
      res.p2[1]=y;

  }


  void Algorithm::load(cv::Mat& depth){
      int i,x,y,ft,*pc;
      double p[3],q[3],a,b,r;
      uchar*pd;

      for (y=0;y<YS;y++)
          {
          pd=depth.ptr<uchar>(y);
          b=depthFOVy*(((double(y)-depthy0)/double(YS-1))-0.5);
          for (x=0;x<XS;x++)
              {
              a=depthFOVx*(((double(x)-depthx0)/double(XS-1))-0.5);
              ft=(int)pd[x];
              r=ft&255;
              r/=255.0;
              if (int(depthcfg&_DepthImage_invert_z)) r=1.0-r;
              r=depthz0+(r*(depthz1-depthz0));

              p[0]=cos(a)*cos(b);
              p[1]=sin(a)*cos(b);
              p[2]=       sin(b);

              if (int(depthcfg&_DepthImage_cos_correct_z)) r/=p[0];
              cloud[y][x].pos[0]=r*p[1];
              cloud[y][x].pos[1]=r*p[2];
              cloud[y][x].pos[2]=r*p[0];
              }
          }

      // compute normals
      for (y=1;y<YS;y++)
         for (x=1;x<XS;x++){
                sub(p,cloud[y][x].pos,cloud[y][x-1].pos);
                sub(q,cloud[y][x].pos,cloud[y-1][x].pos);
                cross(p,q,p);
                mag(cloud[y][x].nor,p);
            }

      // copy missing edge normals
      for (y=1;y<YS;y++) copy(cloud[y][0].nor,cloud[y][1].nor);
      for (x=0;x<XS;x++) copy(cloud[0][x].nor,cloud[1][x].nor);

  }



  bool Algorithm::find_flat_rect(){
        double p[3],n[3];
        y0=0,x0=0;
        x1=0,y1=0;
        double dx=1,dy=1;
        double d;
        int i;

        while(y0<YS){
          x0=0;
          while(x0<XS){
            dx=1;
            dy=1;

            p0=cloud[y0][x0].pos;
            if(p0[2]<error){
              x1=x0+1;
              while(x1<XS){
                p1=cloud[y0][x1].pos;
                if(p1[2]<error){
                  sub(p,p1,p0);
                  d=len(p);
                  if(d>=WX)
                    break;
                }
                x1+=1;
              }

              if(x1>=XS){
                x0=XS;
                continue;
              }
              y1=y0+1;
              while(y1<YS){
                p3=cloud[y1][x0].pos;
                if(p3[2]<error){
                  sub(p,p3,p0);
                  d=len(p);

                  if(d>=WY){
                    break;
                  }
                }
                y1+=1;
              }

              if(y1>=YS){
                p0=p1=p2=p3=NULL;
                return false;
              }

              p2=cloud[y1][x1].pos;
              dx=1+((x1-x0)>>2);
              dy=1+((y1-y0)>>2);

              sub(p,p1,p0);
              sub(n,p3,p0);
              cross(n,n,p);
              mag(n,n);

              i=1;
              for(int y=y0;y<=y1;++y){
                  for(int x=x0;x<=x1;++x){
                      if(cloud[y][x].pos[2]>=error)
                          continue;
                      sub(p,cloud[y][x].pos,p0);
                      d=fabs(dot(p,n));
                      if(d>MAX_VARIANCE){
                          i=0;
                          x=x1+1;
                          y=y1+1;
                          break;
                      }
                  }
              }

              if(i){
                return true;
              }

            }

          x0+=dx;
          }
        y0+=dy;
        }

      return false;
    }

    bool Algorithm::is_clear(){
        int y0=YS/2-RECT_Y/2;
        int x0=XS/2-RECT_X/2;
        int y1=YS/2+RECT_Y/2;
        int x1=XS/2+RECT_X/2;
        double*np0=cloud[y0][x0].pos;
        double*np1=cloud[y0][x1].pos;
        double*np3=cloud[y1][x0].pos;

        double p[3],n[3];
        double d;

        sub(p,p1,p0);
        sub(n,p3,p0);
        cross(n,n,p);
        mag(n,n);

        bool i=true;
        for(int y=y0;y<=y1;++y)
            for(int x=x0;x<=x1;++x){
                if(cloud[y][x].pos[2]>=error)
                    continue;
                sub(p,cloud[y][x].pos,p0);
                d=fabs(dot(p,n));
                if(d>MAX_VARIANCE){
                    i=false;
                    x=x1+1;
                    y=y1+1;
                    break;
                }
            }

        return i;
    }

    bool Algorithm::is_clear_area(cv::Mat&img){
        load(img);
        bool ret=is_clear();
        return ret;
    }

    bool Algorithm::process(cv::Mat&img,ResultPoint&res){
      load(img);
      bool ret=find_flat_rect();
      if(ret){
        project(res);
        return true;
      }
      return false;

    }

}
