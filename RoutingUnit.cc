/*
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Niket Agarwal
 *          Tushar Krishna
 */


#include "mem/ruby/network/garnet2.0/RoutingUnit.hh"

#include "base/cast.hh"
#include "mem/ruby/network/garnet2.0/InputUnit.hh"
#include "mem/ruby/network/garnet2.0/Router.hh"
#include "mem/ruby/slicc_interface/Message.hh"

RoutingUnit::RoutingUnit(Router *router)
{
    m_router = router;
    m_routing_table.clear();
    m_weight_table.clear();
}

void
RoutingUnit::addRoute(const NetDest& routing_table_entry)
{
    m_routing_table.push_back(routing_table_entry);
}

void
RoutingUnit::addWeight(int link_weight)
{
    m_weight_table.push_back(link_weight);
}

/*
 * This is the default routing algorithm in garnet.
 * The routing table is populated during topology creation.
 * Routes can be biased via weight assignments in the topology file.
 * Correct weight assignments are critical to provide deadlock avoidance.
 */

int
RoutingUnit::lookupRoutingTable(int vnet, NetDest msg_destination)
{
    // First find all possible output link candidates
    // For ordered vnet, just choose the first
    // (to make sure different packets don't choose different routes)
    // For unordered vnet, randomly choose any of the links
    // To have a strict ordering between links, they should be given
    // different weights in the topology file

    int output_link = -1;
    int min_weight = INFINITE_;
    std::vector<int> output_link_candidates;
    int num_candidates = 0;

    // Identify the minimum weight among the candidate output links
    for (int link = 0; link < m_routing_table.size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(m_routing_table[link])) {

        if (m_weight_table[link] <= min_weight)
            min_weight = m_weight_table[link];
        }
    }

    // Collect all candidate output links with this minimum weight
    for (int link = 0; link < m_routing_table.size(); link++) {
        if (msg_destination.intersectionIsNotEmpty(m_routing_table[link])) {

            if (m_weight_table[link] == min_weight) {

                num_candidates++;
                output_link_candidates.push_back(link);
            }
        }
    }

    if (output_link_candidates.size() == 0) {
        fatal("Fatal Error:: No Route exists from this Router.");
        exit(0);
    }

    // Randomly select any candidate output link
    int candidate = 0;
    if (!(m_router->get_net_ptr())->isVNetOrdered(vnet))
        candidate = rand() % num_candidates;

    output_link = output_link_candidates.at(candidate);
    return output_link;
}


void
RoutingUnit::addInDirection(PortDirection inport_dirn, int inport_idx)
{
    m_inports_dirn2idx[inport_dirn] = inport_idx;
    m_inports_idx2dirn[inport_idx]  = inport_dirn;
}

void
RoutingUnit::addOutDirection(PortDirection outport_dirn, int outport_idx)
{
    m_outports_dirn2idx[outport_dirn] = outport_idx;
    m_outports_idx2dirn[outport_idx]  = outport_dirn;
}

// outportCompute() is called by the InputUnit
// It calls the routing table by default.
// A template for adaptive topology-specific routing algorithm
// implementations using port directions rather than a static routing
// table is provided here.

int
RoutingUnit::outportCompute(RouteInfo route, int inport,
                            PortDirection inport_dirn)
{
    int outport = -1;

    if (route.dest_router == m_router->get_id()) {

        // Multiple NIs may be connected to this router,
        // all with output port direction = "Local"
        // Get exact outport id from table
        outport = lookupRoutingTable(route.vnet, route.net_dest);
        return outport;
    }

    // Routing Algorithm set in GarnetNetwork.py
    // Can be over-ridden from command line using --routing-algorithm = 1
    RoutingAlgorithm routing_algorithm =
        (RoutingAlgorithm) m_router->get_net_ptr()->getRoutingAlgorithm();

    switch (routing_algorithm) {
        case TABLE_:  outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
        case XY_:     outport =
            outportComputeXY(route, inport, inport_dirn); break;
        // any custom algorithm
        case CUSTOM_: outport =
            outportComputeCustom(route, inport, inport_dirn); break;
        default: outport =
            lookupRoutingTable(route.vnet, route.net_dest); break;
    }

    assert(outport != -1);
    return outport;
}

//valiant routing
int
RoutingUnit::outportComputeXY(RouteInfo route,
                              int inport,
                              PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    int M5_VAR_USED num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);
    
    int random_num = rand() % 2; //generate a random number to select the intermediate group
    int random_group[2]; //use for save those two groups which are not the source or destination group
    int intermediate_y = random_group[random_num]; //the intermediate group
    int y_hop = abs(intermediate_y - my_y);
    bool y_dirn2 = (intermediate_y >= my_y);
    
    int imediate_router = 0;
    
    int x_hop = abs(imediate_router - my_x);
    bool x_dirn2 = (imediate_router >= my_x);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));
    
    
    if (y_hops > 0 && inport_dirn != "South") { // if the destination router and the current router are in the different group and the packet comes from the local channel
        if (inport_dirn == "Local") { // if the packets come from the terminal cores
            for (int i = 0, j = 0; i < 4; i++) {
                if (i != my_y && i != dest_y) {//save two groups which are not the source or destination group
                    random_group[j] = i;
                    j++;
                }
            }
            //choose the router which has the global channel to the intermediate group
            if (y_dirn2){
                switch (y_hop) {
                    case 1:
                        imediate_router = 0;
                        break;
                    case 2:
                        imediate_router = 1;
                    case 3:
                        imediate_router = 2;
                    default:
                        break;
                }
            }
            else{
                switch (y_hop) {
                    case 1:
                        imediate_router = 2;
                        break;
                    case 2:
                        imediate_router = 1;
                    case 3:
                        imediate_router = 0;
                    default:
                        break;
                }
            }
            //choose outport to send packets to the selected router
            if (x_dirn2){
                switch (x_hop){
                    case 1: outport_dirn = "West"; break;
                    case 2: outport_dirn = "North"; break;
                    case 3: outport_dirn = "East"; break;
                    default: break;
                }
            }
            else{
                switch (x_hop) {
                    case 1:
                        outport_dirn = "East";
                        break;
                    case 2:
                        outport_dirn = "North";
                        break;
                    case 3:
                        outport_dirn = "West";
                        break;
                    default:
                        break;
                }
            }
        }
        // the current router has the global channel to the intermediate group
        else{
        outport_dirn = "South";
        }
    }else if (y_hops > 0 && inport_dirn == "South"){ //the current router is in the intermediate group
        //find the router which has connection to the destination group
        if (y_dirn){
            switch (y_hops) {
                case 1:
                    imediate_router = 0;
                    break;
                case 2:
                    imediate_router = 1;
                case 3:
                    imediate_router = 2;
                default:
                    break;
            }
        }
        else{
            switch (y_hops) {
                case 1:
                    imediate_router = 2;
                    break;
                case 2:
                    imediate_router = 1;
                case 3:
                    imediate_router = 0;
                default:
                    break;
            }
        }
        //choose the proper port to send the packets
        if (x_dirn2){
            switch (x_hop){
                case 1: outport_dirn = "West"; break;
                case 2: outport_dirn = "North"; break;
                case 3: outport_dirn = "East"; break;
                default: break;
            }
        }
        else{
            switch (x_hop) {
                case 1:
                    outport_dirn = "East";
                    break;
                case 2:
                    outport_dirn = "North";
                    break;
                case 3:
                    outport_dirn = "West";
                    break;
                default:
                    break;
            }
        }
    }else if (x_hops == 0 && y_hops == 0){
        assert(0);
    }
    else{//the current router and the destination router are in the same group
        if (x_dirn){
            switch (x_hops){
                case 1: outport_dirn = "West"; break;
                case 2: outport_dirn = "North"; break;
                case 3: outport_dirn = "East"; break;
                default: break;
            }
        }
        else{
            switch (x_hops) {
                case 1:
                    outport_dirn = "East";
                    break;
                case 2:
                    outport_dirn = "North";
                    break;
                case 3:
                    outport_dirn = "West";
                    break;
                default:
                    break;
            }
        }
        
    }
   

    return m_outports_dirn2idx[outport_dirn];
}

// Template for implementing custom routing algorithm
// using port directions. (Example adaptive)
int
RoutingUnit::outportComputeCustom(RouteInfo route,
                                 int inport,
                                 PortDirection inport_dirn)
{
    
    assert(0);
    return -1;
}
