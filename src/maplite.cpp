#include <maplite.h>


namespace maplite{


    MapliteClass::MapliteClass(ros::NodeHandle& n){

        //-1. get parameters
        if(!ros::param::get("/maplite/maplite_pandemonium/lon_zero", lon_zero_)){
            ROS_ERROR("[maplite] no initial guess lon");
        }
        if(!ros::param::get("/maplite/maplite_pandemonium/lat_zero", lat_zero_)){
            ROS_ERROR("[maplite] no initial guess lat");
        }


        //0. get the nodes
        osm_node_client_ = n.serviceClient<osm_planner::osmWays>("/ways_serv", true);



        //0.a iterate through all the ways 
        osm_planner::osmWays srv;

        while(!osm_node_client_.call(srv)){

            usleep(100);
        }


        OSM_WAY way_now; 
        for(int i = 0; i < srv.response.size; ++i){

            double lon = (srv.response.longitude[i]);
            double lat = (srv.response.latitude[i]);

            if(lon == 0 && lat == 0){
                ways_.push_back(way_now);
                way_now.clear();
            }
            else{
                way_now.push_back({(lat-lat_zero_)*METERS_PER_LAT, (lon - lon_zero_)*METERS_PER_LON});
            }

        }            

        //1. get localization points


        //2.localize
        this->localize();
    }

    void MapliteClass::printWays(std::vector<OSM_WAY> & vec){

        std::cout << "Way count : " << vec.size() << std::endl;
        int i = 0;
        for(OSM_WAY w :vec){

            std::cout << "  Way [" << i++ << "] with size ["<< 
                    w.size()  << "] : " << std::endl;

            for(COORD c : w){
                std::cout << "      (" << std::setprecision(9)<<
                        c.y << ","   << std::setprecision(9) <<
                        c.x << ")\n";
            }

            std::cout<< std::endl;
            return;
        }
    }


    void MapliteClass::localize(void){

        printWays(ways_);
        double epsilon = 5;

        for(int way_id = 0; way_id < ways_.size(); ++way_id){
            filtered_ways_.push_back
                (applyDouglasPeuker(way_id, epsilon, 0, ways_[way_id].size() -1));
                break;
        }

        printWays(filtered_ways_);
    }

    OSM_WAY MapliteClass::applyDouglasPeuker(uint way_id, double epsilon, uint start, uint end){

        double dmax = 0;
        uint index = 0;
        COORD st_coord  = ways_[way_id][start];
        COORD end_coord = ways_[way_id][end];

        //0. Find maximum distance point 
        for(int i = start+1; i < end; ++i){

            double d = perpendicularDist(ways_[way_id][i], st_coord, end_coord);

            if(d>dmax){
                dmax = d;
                index = i;
            }
        }
        //1. apply logic of epsilon 
        OSM_WAY    ans;

        //1(a) Distance is too far to create one line. Recurse
        if(dmax>epsilon){
            OSM_WAY recRes1 = applyDouglasPeuker(way_id, epsilon, start, index);
            OSM_WAY recRes2 = applyDouglasPeuker(way_id, epsilon, index, end);

            ans.insert(ans.end(), recRes1.begin(), recRes1.end());
            ans.pop_back();
            ans.insert(ans.end(), recRes2.begin(), recRes2.end());
        } 
        //2(b) epsilon is never met. Drop all in-between points
        else{

            ans = {st_coord, end_coord};
        }

        return ans;
    }

    double MapliteClass::perpendicularDist(COORD p, COORD start, COORD end){

        double m = ( end.y - start.y)/ (end.x - start.x);
        double b = end.y - m*start.x;

        // |Ax + By + c|/sqrt(A^2 + B^2)
        return abs(-p.y + m*p.x + b)/sqrt(1 + m*m);
    }

}