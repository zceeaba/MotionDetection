//
// Created by abhishekh.baskaran on 26/03/2019.
//

#include "inc/signalHandler.h"

/**
 * @brief Signal Handler constructor to set up different signals
 */
SignalHandler::SignalHandler() {

    /// Define different signals to handle
    signal(SIGINT, signalHandlerGeneral);
    signal(SIGILL, signalHandlerGeneral);
    signal(SIGSEGV, signalHandlerGeneral);
    signal(SIGTERM, signalHandlerGeneral);
    signal(SIGABRT, signalHandlerGeneral);
    signal(SIGFPE, signalHandlerGeneral);

}

/**
 * @brief Provides a clean exit while handling the signal and displaying the signal name
 * @param signum : Value of signal to handle
 */
void SignalHandler::signalHandlerGeneral(int signum) {
    /// Map each signal key to its name
    std::map<int,std::string> signalNamesMap;

    /// Generate Map for the signal handlers
    signalNamesMap.insert(std::make_pair(SIGINT, "Interrupt"));
    signalNamesMap.insert(std::make_pair(SIGILL, "Illegal Instruction:SIGILL"));
    signalNamesMap.insert(std::make_pair(SIGSEGV, "Segmentation Fault:SIGSEGV"));
    signalNamesMap.insert(std::make_pair(SIGTERM, "Terminate:SIGTERM"));
    signalNamesMap.insert(std::make_pair(SIGABRT, "Abort:SIGABRT"));
    signalNamesMap.insert(std::make_pair(SIGFPE, "FPE:SIGFPE"));

    std::cout << "signal (" << signalNamesMap.at(signum) << ") received." << std::endl;
    std::cout << "Exiting cleanly";

    /// cleanup and close up stuff here
    /// terminate program

    std::exit(signum);
}




