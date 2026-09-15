#ifndef VAG_ODO_FUEL_H_INCLUDED
#define VAG_ODO_FUEL_H_INCLUDED

class CBuffer;

// VAG (VW/Audi Group) UDS odometer + fuel-level polling, specific to the
// Passat B8's gateway/instrument-cluster addressing (0x710/0x77A, DID
// 0x02BD for odometer; 0x714/0x77E, DID 0x22B0 for fuel level - see
// vag_odo_fuel.cpp for the full HexSniff-derived reasoning). Guarded by
// ENABLE_VAG_ODO_FUEL (config.h) - compiled out entirely for other
// vehicles (e.g. the Zafira/PSA build, see platformio.ini env:esp32dev_zafira).
void processVagOdoFuel(CBuffer* buffer);

#endif
