#include "safe_takeoff/algorithm.h"
#include "safe_takeoff/helper.h"

#include <string.h>
#include <algorithm>
#include <vector>
#include<iostream>

namespace safe_takeoff{

    void Algorithm::construct(std::vector<std::vector<double> >&mesh){
        int ys=mesh.size();
        int xs=mesh[0].size();

        for(int y=0; y<ys; ++y)
            for(int x=0; x<xs; ++x){
                //std::cout<<mesh[y][x]<<" ";
                  if(mesh[y][x]>MIN_ALT && mesh[y][x]<MAX_ALT)
                      measure[y][x]=1;
                  else measure[y][x]=0;
            }
    }

    bool Algorithm::search(std::vector<std::vector<double> >&mesh,
                          ResultPoint&res){
        int ys=mesh.size();
        int xs=mesh[0].size();
        int max_of_s = 0, max_i = 0, max_j = 0;
        memset(dp, 0, sizeof dp);

        for(int y = 1; y < ys; y++){
            for(int x = 1; x < xs; x++){
              if( measure[y-1][x-1]==1)
                   dp[y][x] = std::min(dp[y-1][x],
                              std::min(dp[y][x-1],
                              dp[y-1][x-1] )) + 1;

              if( dp[y][x] > max_of_s ){
                  max_of_s = dp[y][x];
                  max_i=x;
                  max_j=y;
                }
            }
        }

        if(max_of_s>MIN_WINDOW){
            res.p0[0]=max_j-max_of_s;
            res.p0[1]=max_i-max_of_s;
            res.p2[0]=max_j;
            res.p2[1]=max_i;

            return true;
        }
        return false;
    }


    bool Algorithm::search_rect(){
        int y0=YS/2-RECT_Y/2;
        int x0=XS/2-RECT_X/2;
        int y1=YS/2+RECT_Y/2;
        int x1=XS/2+RECT_X/2;
        int ny=y1-y0;
        int nx=x1-x0;

        int error=0;
        // searching through the whole array as mesh is constructed
        // in a way to only contain the area above the drone 
        // for performance boosting
        for(int y=0; y<ny; ++y)
            for(int x=0; x<nx; ++x)
                if(measure[y][x]==1)
                    error+=1;
        std::cout<<"error= "<<error<<std::endl;
        return error<MAX_ERROR;
    }

    bool Algorithm::is_clear_area(std::vector<std::vector<double> >&mesh){
        construct(mesh);
        //std::cout<<"finsish contruct"<<std::endl;
        bool ret=search_rect();
        return ret;
    }


    bool Algorithm::process(std::vector<std::vector<double> >&mesh,
                            ResultPoint&res){
        construct(mesh);
        bool ret=search(mesh,res);
        return ret;
    }

}
