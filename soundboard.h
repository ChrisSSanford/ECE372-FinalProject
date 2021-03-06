/**************************************************************************************************/

/*
 * File:    soundboard.h
 * Team:    Lambda^3
 * Members: Chris Houseman
 *          Randy Martinez
 *          Rachel Powers
 *          Chris Sanford
 *
 * Date: October 2, 2014
 *
 * Description: Header file for soundboard functions.
 *
 */

// ******************************************************************************************* //
void SBInitialize();
void SBReset();
void SBPlayVoice(int voiceNumber);
void SBAsyncPlayVoice(int voiceNumber);
void SBStopVoice();
void SBPauseVoice();
void SBMute();
void SBUnmute();
void SBSendCommand(unsigned int command);

// ******************************************************************************************* //
void Delayms(unsigned int msDelay);