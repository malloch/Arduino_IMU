Spine
=========

Arduino sensor-fusion firmware for estimating orientation using accelerometers, rate-gyroscopes, and magnetometers.
This firmware calculates and outputs an orientation quaternion using a complementary filter.

This firmware and software was written for a digital musical instrument --- "The Spine" --- developed for the collaborative research-creation project [*Les Gestes*][1].
The Spine contains at least two IMUs for calculating orientation and deformation in real-time - a video demonstrating an early prototype can be found [here][2].

Tested with the [Mongoose IMU][3], but the sensor fusion code should be easy to adapt to other sensor platforms.

Except where otherwise noted, this software and firmware is licensed with the GPLv3; see the attached file
COPYING for details, which should be included in this download.

Joseph Malloch 2011
[Input Devices and Music Interaction Laboratory][4], McGill University.

[1]: http://idmil.org/projects/gestes
[2]: http://www.youtube.com/watch?v=-Dqvf1CXPWg
[3]: http://store.ckdevices.com/products/Mongoose-9DoF-IMU-with-Barometric-Pressure-Sensor-.html
[2]: http://idmil.org

