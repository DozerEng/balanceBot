/*!
 * \class BalanceBot
 * \file BalanceBot.cpp
 * \author Michael Pillon
 * 
 */

#include "BalanceBot.hpp"

BalanceBot::BalanceBot(A4988* lw, A4988* rw) : 
    leftWheel(lw),
    rightWheel(rw)
    {

}

/*!
    Incrememnt both wheels forward
    \param count Number of steps to rotate wheels
*/
void BalanceBot::step(const uint8_t count) {
    for(int i = 0; i < count; i++) {
        leftWheel->setStep(HIGH);
        rightWheel->setStep(HIGH);
        ThisThread::sleep_for(STEP_DELAY);
        leftWheel->setStep(LOW);
        rightWheel->setStep(LOW);
        ThisThread::sleep_for(STEP_DELAY);
    }
}
/*!
    Set direction of both wheels
    \param dir Direction to set wheels. No arg toggles state
 */
void BalanceBot::setDirection() {
    leftWheel->setDirMode();
    rightWheel->setDirMode();
}

void BalanceBot::setDirection(const uint8_t dir) {
    leftWheel->setDirMode(dir);
    rightWheel->setDirMode(dir);
}
/*!
    Set wheel motor microstepping mode
    Calls corresponding A4988 stepMode function
*/
void BalanceBot::setStepMode(const uint8_t mode) {
    leftWheel->setStepMode(mode);
    rightWheel->setStepMode(mode);
}
void BalanceBot::incStepMode() {
    leftWheel->incStepMode();
    rightWheel->incStepMode();
}
void BalanceBot::decStepMode() {
    leftWheel->decStepMode();
    rightWheel->decStepMode();
}