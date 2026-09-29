"""Assetto Corsa shared-memory pages.

The layout matches the Windows pages the game writes. wchar fields are
UTF-16 so the offsets stay correct when this runs on Linux.
"""

import ctypes
from ctypes import c_float, c_int32, c_uint16


def _w(count):
    return c_uint16 * count


class PhysicsPage(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ('packet_id', c_int32),
        ('gas', c_float),
        ('brake', c_float),
        ('fuel', c_float),
        ('gear', c_int32),
        ('rpms', c_int32),
        ('steer_angle', c_float),
        ('speed_kmh', c_float),
        ('velocity', c_float * 3),
        ('acc_g', c_float * 3),
        ('wheel_slip', c_float * 4),
        ('wheel_load', c_float * 4),
        ('wheels_pressure', c_float * 4),
        ('wheel_angular_speed', c_float * 4),
        ('tyre_wear', c_float * 4),
        ('tyre_dirty_level', c_float * 4),
        ('tyre_core_temperature', c_float * 4),
        ('camber_rad', c_float * 4),
        ('suspension_travel', c_float * 4),
        ('drs', c_float),
        ('tc', c_float),
        ('heading', c_float),
        ('pitch', c_float),
        ('roll', c_float),
        ('cg_height', c_float),
        ('car_damage', c_float * 5),
        ('number_of_tyres_out', c_int32),
        ('pit_limiter_on', c_int32),
        ('abs', c_float),
        ('kers_charge', c_float),
        ('kers_input', c_float),
        ('auto_shifter_on', c_int32),
        ('ride_height', c_float * 2),
        ('turbo_boost', c_float),
        ('ballast', c_float),
        ('air_density', c_float),
        ('air_temp', c_float),
        ('road_temp', c_float),
        ('local_angular_vel', c_float * 3),
        ('final_ff', c_float),
        ('performance_meter', c_float),
        ('engine_brake', c_int32),
        ('ers_recovery_level', c_int32),
        ('ers_power_level', c_int32),
        ('ers_heat_charging', c_int32),
        ('ers_is_charging', c_int32),
        ('kers_current_kj', c_float),
        ('drs_available', c_int32),
        ('drs_enabled', c_int32),
        ('brake_temp', c_float * 4),
        ('clutch', c_float),
        ('tyre_temp_i', c_float * 4),
        ('tyre_temp_m', c_float * 4),
        ('tyre_temp_o', c_float * 4),
        ('is_ai_controlled', c_int32),
        ('tyre_contact_point', c_float * 4 * 3),
        ('tyre_contact_normal', c_float * 4 * 3),
        ('tyre_contact_heading', c_float * 4 * 3),
        ('brake_bias', c_float),
        ('local_velocity', c_float * 3),
    ]


class GraphicsPage(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ('packet_id', c_int32),
        ('status', c_int32),
        ('session', c_int32),
        ('current_time', _w(15)),
        ('last_time', _w(15)),
        ('best_time', _w(15)),
        ('split', _w(15)),
        ('completed_laps', c_int32),
        ('position', c_int32),
        ('i_current_time', c_int32),
        ('i_last_time', c_int32),
        ('i_best_time', c_int32),
        ('session_time_left', c_float),
        ('distance_traveled', c_float),
        ('is_in_pit', c_int32),
        ('current_sector_index', c_int32),
        ('last_sector_time', c_int32),
        ('number_of_laps', c_int32),
        ('tyre_compound', _w(33)),
        ('replay_time_multiplier', c_float),
        ('normalized_car_position', c_float),
        ('car_coordinates', c_float * 3),
    ]


def physics_from_bytes(buf: bytes):
    if len(buf) < ctypes.sizeof(PhysicsPage):
        return None
    return PhysicsPage.from_buffer_copy(buf[:ctypes.sizeof(PhysicsPage)])


def graphics_from_bytes(buf: bytes):
    if len(buf) < ctypes.sizeof(GraphicsPage):
        return None
    return GraphicsPage.from_buffer_copy(buf[:ctypes.sizeof(GraphicsPage)])
