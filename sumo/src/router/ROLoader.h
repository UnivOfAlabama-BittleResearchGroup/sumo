#ifndef ROLoader_h
#define ROLoader_h
//---------------------------------------------------------------------------//
//                        ROLoader.h -
//  Loader for networks and route imports
//                           -------------------
//  project              : SUMO - Simulation of Urban MObility
//  begin                : Sept 2002
//  copyright            : (C) 2002 by Daniel Krajzewicz
//  organisation         : IVF/DLR http://ivf.dlr.de
//  email                : Daniel.Krajzewicz@dlr.de
//---------------------------------------------------------------------------//

//---------------------------------------------------------------------------//
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
//---------------------------------------------------------------------------//
// $Log$
// Revision 1.7  2004/07/02 09:39:41  dkrajzew
// debugging while working on INVENT; preparation of classes to be derived for an online-routing
//
// Revision 1.6  2004/04/14 13:53:50  roessel
// Changes and additions in order to implement supplementary-weights.
//
// Revision 1.5  2004/02/16 13:47:07  dkrajzew
// Type-dependent loader/generator-"API" changed
//
// Revision 1.4  2004/01/26 08:01:10  dkrajzew
// loaders and route-def types are now renamed in an senseful way; further changes in order to make both new routers work; documentation added
//
// Revision 1.3  2003/08/18 12:44:54  dkrajzew
// xerces 2.2 and later compatibility patched
//
// Revision 1.2  2003/02/07 10:45:07  dkrajzew
// updated
//
/* =========================================================================
 * included modules
 * ======================================================================= */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H

#include <string>
#include "RORouteDefList.h"
#include <sax2/SAX2XMLReader.hpp>


/* =========================================================================
 * xerces 2.2 compatibility
 * ======================================================================= */
#if defined(XERCES_HAS_CPP_NAMESPACE)
using namespace XERCES_CPP_NAMESPACE;
#endif


/* =========================================================================
 * class declarations
 * ======================================================================= */
class OptionsCont;
class RONet;
class RONetHandler;
class ROAbstractRouteDefLoader;
class ofstream;
class GenericSAX2Handler;
class ROAbstractRouter;
class ROAbstractEdgeBuilder;
class ROVehicleBuilder;


/* =========================================================================
 * class definitions
 * ======================================================================= */
/**
 * @class ROLoader
 * The data loader. Loads the network and route descriptions using further
 * classes. Is capable to either load all routes in one step or go through
 * them step wise.
 */
class ROLoader {
public:
    /// Constructor
    ROLoader(OptionsCont &oc, ROVehicleBuilder &vb,
        bool emptyDestinationsAllowed);

    /// Destructor
    virtual ~ROLoader();

    /// Loads the network
    virtual RONet *loadNet(ROAbstractEdgeBuilder &eb);

    /// Loads the net weights
    bool loadWeights(RONet &net);

    /// Parse the supplementary-weights-file. This will add
    /// supplementary weights to the RONet's ROEdges.
    ///
    /// @param net The net to which the weights should be aaded.
    ///
    /// @return True on successful parsing.
    ///
    bool loadSupplementaryWeights( RONet& net );

    /** @brief Builds and opens all route loaders
        Route loaders are derived from ROAbstractRouteDefLoader */
    void openRoutes(RONet &net, float gBeta, float gA);

    /** @brief Loads routes stepwise
        This is done for all previously build route loaders */
    virtual void processRoutesStepWise(long start, long end,
        RONet &net, ROAbstractRouter &router);

    /** @brief Loads all routes at once
        This is done for all previously build route loaders */
    virtual void processAllRoutes(unsigned int start, unsigned int end,
        RONet &net, ROAbstractRouter &router);

    /** @brief Ends route reading
        This is done for all previously build route loaders */
    void closeReading();

protected:
    /** @brief Loads the net
        The loading structures were built in previous */
/*    virtual bool loadNet(SAX2XMLReader *reader, RONetHandler &handler,
        const std::string &files);*/

    /** @brief Opens routes
        The loading structures were built in previous */
    void openTypedRoutes(const std::string &optionName,
        RONet &net, float gBeta, float gA);

    /// Adds a route loader to the list of known route loaders
    void addToHandlerList(const std::string &optionName,
        RONet &net, float gBeta, float gA);

    /** @brief Skips routes which start before the wished time period
        This is done for all previously build route loaders */
    void skipUntilBegin();

    /// Returns the first known time step
    unsigned int getMinTimeStep() const;

protected:
    ROAbstractRouteDefLoader* buildNamedHandler(
        const std::string &optionName, const std::string &file
        , RONet &net, float gBeta, float gA);

    void checkFile(const std::string &optionName
        , const std::string &file);

    void writeStats(int time, int start, int absNo);


protected:
    /// Options to use
    OptionsCont &_options;

    /// Definition of route loader list
    typedef std::vector<ROAbstractRouteDefLoader*> RouteLoaderCont;

    /// List of route loaders
    RouteLoaderCont _handler;

    /// Information whether empty destinations are allowed
    bool myEmptyDestinationsAllowed;

    /// The vehicle builder to use
    ROVehicleBuilder &myVehicleBuilder;

private:
    /// invalidated copy constructor
    ROLoader(const ROLoader &src);

    /// invalidated assignment operator
    ROLoader &operator=(const ROLoader &src);

};


/**************** DO NOT DEFINE ANYTHING AFTER THE INCLUDE *****************/

#endif

// Local Variables:
// mode:C++
// End:

