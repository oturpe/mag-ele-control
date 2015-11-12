#ifndef MAGNET_DRIVER_H
#define MAGNET_DRIVER_H

#include <stdint.h>

/// \class MagnetDriver
///
/// Drivers the magnets of the device with given strength
class MagnetDriver {

public:
    /// \brief
    ///    Initialized a new magnet driver instance
    MagnetDriver();

    /// \brief
    ///    Destroys this magnet driver instance
    ~MagnetDriver();

public:
    /// \brief
    ///    Sets the magnets to given strengths
    ///
    /// Values are given as array of values, one for each of the three magnets
    ///
    /// \param values
    ///    Values to set
    void set(uint8_t * values);
};

#endif
