/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_Compass_TEST_H
#define AP_Compass_TEST_H

#include <AP_Compass.h>

class AP_Compass_TEST : public AP_Compass_Backend
{
public:
    AP_Compass_TEST(AP_Compass &_compass);
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

    // return the number of compass instances
    uint8_t get_count(void) const { return _num_instances; }

    // return the primary compass
    uint8_t get_primary(void) const;

private:
    uint8_t _num_instances;
    int _mag_fd[COMPASS_MAX_INSTANCES];
    Vector3f _sum[COMPASS_MAX_INSTANCES];
    uint32_t _count[COMPASS_MAX_INSTANCES];
    uint64_t _last_timestamp[COMPASS_MAX_INSTANCES];
    bool _is_external[COMPASS_MAX_INSTANCES];
};

#endif // AP_Compass_PX4_H

