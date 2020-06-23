#ifndef INC_SIGNALHANDLER_H
#define INC_SIGNALHANDLER_H

//
// Created by abhishekh.baskaran on 26/03/2019.
//
#include <csignal>
#include <map>
#include <iostream>



/**
 * @class SignalHandler
 * @brief Utility class to perform signal handling and intercept termination/interrupt signals
 */

class SignalHandler {
public:
    /**
     * @brief Default constructor to initialize signal handler
     */
    SignalHandler();
    /**
     * @brief Signal handler to handle interrupts
     * @param signum
     */
    static void signalHandlerGeneral(int signum);

    virtual ~SignalHandler()= default;

};


#endif //INC_SIGNALHANDLER_H
