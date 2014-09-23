/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_Compass_PX4_H
#define AP_Compass_PX4_H

#include <AP_Compass.h>

class AP_Compass_PX4 : public AP_Compass_Backend
{
public:
    AP_Compass_PX4(AP_Compass &_compass, AP_Compass::Compass_State &_state);
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

    // return the number of compass instances
    uint8_t get_count(void) const { return _num_instances; }

    // return the primary compass
    uint8_t get_primary(void) const;

private:
    uint8_t _num_instances;
    int _mag_fd;
    Vector3f _sum;
    uint32_t _count;
    uint64_t _last_timestamp;
    bool _is_external;
};

#endif // AP_Compass_PX4_H

