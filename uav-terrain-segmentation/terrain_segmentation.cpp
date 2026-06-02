#include <chrono>
using namespace std::chrono;
using namespace std;

#include"math.cpp"

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

const int _DepthImage_invert_z      =0x00000001;    // brighter colros are closer
const int _DepthImage_cos_correct_z =0x00000002;    // depth is perpendicular distance instead of euclid
const int _DepthImage_smooth        =0x00000004;    // average position to reduce noise
const double deg=M_PI/180.0;
const int YS=720;
const int XS=1280;

class DepthPoint
    {
public:
    double pos[3];  // position (x,y,z)
    double nor[3];  // normal vector (x,y,z)

    DepthPoint(){};
    DepthPoint* operator = (const DepthPoint *a) { *this=*a; return this; }
    };

class DepthImage
    {
public:
    // resoluton
    int xs,ys;                  // resolution
    // point cloud ys x xs
    std::vector<std::vector<DepthPoint> >cloud;           // cloud[y][x] point for each pixel(x,y) of image
    cv::Mat color,depth;
    // depth camera properties
    double depthFOVx,depthFOVy, // [rad]    FOV angles
           depthx0,depthy0,     // [pixels] FOV offset
           depthz0,depthz1;     // [m] intensity <0,255> -> distance <z0,z1>
    int    depthcfg;            // configuration flags
    double error;               // invalid depth


    DepthImage()
        {
        xs=XS;
        ys=YS;
        depthFOVx=86.0*deg;
        depthFOVy=57.0*deg;
        depthx0=0.5;
        depthy0=0.5;
        depthz0=0.8;
        depthz1=1.;
        error=depthz1*0.99;
        depthcfg=_DepthImage_invert_z|_DepthImage_cos_correct_z;
        cloud=std::vector<std::vector<DepthPoint> >(YS,std::vector<DepthPoint>(XS));
        }

    DepthImage(DepthImage& a)   { *this=a; }
    ~DepthImage()   { std::vector<std::vector<DepthPoint> >().swap(cloud); }
    DepthImage* operator = (const DepthImage *a) { *this=*a; return this; }
    //DepthImage* operator = (const DepthImage &a) { ...copy... return this; }


    void draw(double*p0,double*p2){
      double x=atan2(p0[0],p0[2])/depthFOVx;
      double y=atan2(p0[1],p0[2])/depthFOVy;
      if(int(depthcfg&_DepthImage_cos_correct_z))
          y*=(ys*depthFOVx)/(depthFOVy*xs);

      x=(x+0.5)*xs;
      y=(y+0.5)*ys;
      //std::cout<<x<<' '<<y<<std::endl;

      double x2=atan2(p2[0],p2[2])/depthFOVx;
      double y2=atan2(p2[1],p2[2])/depthFOVy;
      if(int(depthcfg&_DepthImage_cos_correct_z))
          y2*=(ys*depthFOVx)/(depthFOVy*xs);

      x2=(x2+0.5)*xs;
      y2=(y2+0.5)*ys;
      //std::cout<<x2<<' '<<y2<<std::endl;

      cv::rectangle(color,cv::Point(x,y),cv::Point(x2,y2),cv::Scalar(0, 255, 0));
      cv::imshow("",color);
      cv::waitKey(0);
    }

    void load(std::string img_path)
        {
        int i,x,y,ft,*pc;
        double p[3],q[3],a,b,r;
        uchar*pd;
        // load depth image
        depth=cv::imread(img_path,cv::IMREAD_GRAYSCALE);
        color=cv::imread(img_path);

        for (y=0;y<ys;y++)
            {
            pd=depth.ptr<uchar>(y);                // pointer to y-th scan line in depth image
            b=depthFOVy*(((double(y)-depthy0)/double(ys-1))-0.5);       // latitude
            for (x=0;x<xs;x++)
                {
                a=depthFOVx*(((double(x)-depthx0)/double(xs-1))-0.5);   // longitude
                //std::cout<<a<<' '<<b<<std::endl;
                ft=(int)pd[x];
                r=ft&255;                            // RGB from drepth (just single channel
                r/=255.0;                               // linear scale
                if (int(depthcfg&_DepthImage_invert_z)) r=1.0-r;
                r=depthz0+(r*(depthz1-depthz0));        // range <z0,z1>
                // spherical -> cartesian
                p[0]=cos(a)*cos(b);         // x
                p[1]=sin(a)*cos(b);         // y
                p[2]=       sin(b);         // z
                //std::cout<<p[0]<<' '<<p[1]<<' '<<p[2]<<std::endl;

                if (int(depthcfg&_DepthImage_cos_correct_z)) r/=p[0];
                // scale and reorder coordinates to match my view
                cloud[y][x].pos[0]=r*p[1];
                cloud[y][x].pos[1]=r*p[2];
                cloud[y][x].pos[2]=r*p[0];
                //std::cout<<cloud[y][x].pos[0]<<' '<<cloud[y][x].pos[1]<<' '<<cloud[y][x].pos[2]<<std::endl;
                }
            }

        // compute normals
        for (y=1;y<ys;y++)
         for (x=1;x<xs;x++)
            {
            sub(p,cloud[y][x].pos,cloud[y][x-1].pos);// p = cloud[y][x] - cloud[y][x-1]
            sub(q,cloud[y][x].pos,cloud[y-1][x].pos);// q = cloud[y][x] - cloud[y-1][x]
            cross(p,q,p);                          // p = cross(q,p)
            //std::cout<<p[0]<<'\t'<<p[1]<<'\t'<<p[2]<<std::endl;
            mag(cloud[y][x].nor,p);                // nor[y][x] = p/|p|
            //std::cout<<cloud[y][x].nor[0]<<' '<<cloud[y][x].nor[1]<<' '<<cloud[y][x].nor[2]<<std::endl;
            }
        // copy missing edge normals
        for (y=1;y<ys;y++) copy(cloud[y][0].nor,cloud[y][1].nor);
        for (x=0;x<xs;x++) copy(cloud[0][x].nor,cloud[1][x].nor);


        /*
        for(int y=0;y<ys;++y)
          for(int x=0;x<xs;++x)
            std::cout<<cloud[y][x].pos[0]<<' '<<cloud[y][x].pos[1]<<' '<<cloud[y][x].pos[2]<<std::endl;
          */
}


void load_from_file(std::string file_name){
  std::ifstream f(file_name);
  std::string line;
  for(int y=0; y<ys; ++y){
    for(int x=0; x<xs; ++x){
      std::getline(f,line);
      std::vector<double>pos;
      std::stringstream ss(line);
      for (double i; ss >> i;) {
          pos.push_back(i);
          if (ss.peek() == ',')
              ss.ignore();
      }

      for(int i=0; i<3; ++i)
        cloud[y][x].pos[i]=pos[i];

      std::getline(f,line);
      std::vector<double>nor;
      ss=std::stringstream(line);
      for (double i; ss >> i;) {
          nor.push_back(i);
          if (ss.peek() == ',')
              ss.ignore();
      }

      for(int i=0; i<3; ++i)
        cloud[y][x].nor[i]=nor[i];
    }

  }


}


    bool find_flat_rect(double* &p0,double* &p1,double* &p2,double* &p3,double wx,double wy,double dmax)
        {
          double p[3],n[3];
          int y0=0,x0=0;
          int x1=0,y1=0;
          double dx=1,dy=1;
          double d;
          int i;

          while(y0<ys){
            x0=0;
            while(x0<xs){
              //std::cout<<y0<<' '<<x0<<std::endl;
              dx=1;
              dy=1;

              p0=cloud[y0][x0].pos;
              if(p0[2]<error){
                x1=x0+1;
                while(x1<xs){
                  p1=cloud[y0][x1].pos;
                  if(p1[2]<error){
                    sub(p,p1,p0);
                    d=len(p);
                    if(d>=wx)
                      break;
                  }
                  x1+=1;
                }

                if(x1>=xs){
                  x0=xs;
                  continue;
                }
                y1=y0+1;
                while(y1<ys){
                  p3=cloud[y1][x0].pos;
                  if(p3[2]<error){
                    sub(p,p3,p0);
                    d=len(p);

                    if(d>=wy){
                      //std::cout<<"---------- "<<y0<<' '<<x0<<' '<<x1<<' '<<y1<<' '<<d<<std::endl;
                      break;
                      //std::cout<<444444<<std::endl;
                    }
                  }
                  y1+=1;

                }
                if(y1>=ys){
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
                //std::cout<<"-------\n";
                i=1;
                for(int y=y0;y<=y1;++y){
                  for(int x=x0;x<=x1;++x){
                    if(cloud[y][x].pos[2]>=error)
                      continue;
                    sub(p,cloud[y][x].pos,p0);
                    d=fabs(dot(p,n));
                    if(d>dmax){
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

        std::cout<<"couldn't find solution\n";
        return false;
        }
    };

int main(){

  DepthImage pcl;         // pointcloud
  double *q0,*q1,*q2,*q3; // pointers to landing zone rectangle points
  std::string img_path="/media/bignrz/Fast Data/projects/Drone/code/safe_landing test/console/src/338.jpg";

  pcl.load(img_path); // this creates the pointcloud from image

  auto start = high_resolution_clock::now();
  auto ret=pcl.find_flat_rect(q0,q1,q2,q3,0.4,0.4,0.005);
  if(ret)
    pcl.draw(q0,q2);
  else
    std::cout<<"couldn't find solution\n";

  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  //std::cout << duration.count() << std::endl;
}
