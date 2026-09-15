#ifndef PSA_ODO_FUEL_H_INCLUDED
#define PSA_ODO_FUEL_H_INCLUDED

class CBuffer;

// PSA/Stellantis (K0 platform: Peugeot Traveller/Expert, Citroen SpaceTourer/
// Jumpy, Opel/Vauxhall Zafira Life/Vivaro Life, Toyota Proace Verso) UDS
// odometer polling. PSA addressing differs from VAG: request ID = response
// ID + 0x100 (VAG uses +8), and reads need session 1003 but NO security
// access (0x27 is only required before writes) - see psa_odo_fuel.cpp for
// the full sourcing (PyPSADiag / arduino-psa-diag ECU_LIST.md).
//
// Fuel level has NO confirmed UDS DID as of 2026-09-15 - only configuration
// DIDs (gauge algorithm, sender resistance calibration) were found, not a
// live reading. This module instead does an exploratory raw capture of DID
// 0x2100/0x2101 ("gauging group" descriptor + data) so real-drive data can
// be gathered now and decoded later, same approach used for the VAG fuel
// DID before it was confirmed.
void processPsaOdoFuel(CBuffer* buffer);

#endif
