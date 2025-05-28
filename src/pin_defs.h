#ifndef PIN_DEFS_H
#define PIN_DEFS_H

#warning "USANDO PIN_DEFS.H (SOLO PARA PCB FINAL)"

#define PCEN 	4u		// Payload Camera "Enable"
#define GCEN 	5u		// Ground Camera "Enable"
#define TIP_AG 	20u		// TIP base AutoGyro Deployment
#define TIP_PA 	21u		// TIP base Payload Deployment
#define D3_RX 	3u		// -> TX GPS
#define D2_TX 	2u		// -> RX GPS
#define S1_A1	15u		// IRQ RPM
#define S2_A2	16u		// IRQ RPM
#define CS_M 	14u 	// CS Memory
#define CS_E 	10u 	// CS Encoder
#define A3_BAT 	17u 	// VBAT Sensor (scaled)
#define ENA_M 	9u		// Motor Enable
#define IN1_M	8u		// Motor IN1
#define IN2_M	7u		// Motor IN2
#define IN3_M	6u		// Motor IN3


#endif // PIN_DEFS_H