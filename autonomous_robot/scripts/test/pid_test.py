#!/usr/bin/env python

'''def self.pid_control(kp, ki, kd, rk, yk, ts):
    ek = rk - yk
    ik = self.ik_1+ts*(ek + self.ek_1)/2
    dk = (ek - self.ek_1)/ts
    uk = kp*ek + ki*ik + kd*dk
    self.ek_1 = ek
    self.ik_1 = ik
    return uk'''
