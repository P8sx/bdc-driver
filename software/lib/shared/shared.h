/*
 * shared.h
 *
 *  Created on: Jun 15, 2025
 *      Author: phsx
 */

#ifndef SHARED_SHARED_H_
#define SHARED_SHARED_H_

#define BUILD_DATETIME   						__DATE__ " " __TIME__

#ifdef BOOTLOADER
	#define BUILD_VERSION						"0.1.0"
#elif defined(APP)
	#define BUILD_VERSION						"1.0.0"
#endif

#define BOOT_APP_ADDRESS 			   			0x08007800

#define BOOTFLAG_BOOTLOADER			  			0xB007B007
#define BOOTFLAG_APP				  			0xB007AAAA
#define BOOTFLAG_APP_OTW			  			0xB007C007


#endif /* SHARED_SHARED_H_ */
