# StrongArm

A strong robot arm with two MyActuator motors.

[https://medium.com/@TomPJacobs/build-a-robot-9fc864318622](https://medium.com/@TomPJacobs/build-a-robot-9fc864318622)

[https://medium.com/p/dfd7fef49c1a](https://medium.com/@TomPJacobs/build-a-robot-part-ii-dfd7fef49c1a)


Also, tip for removing the drive not ejected messages, run this, and restart your computer:

```sudo defaults write /Library/Preferences/SystemConfiguration/com.apple.DiskArbitration.diskarbitrationd.plist DADisableEjectNotification -bool YES && sudo pkill diskarbitrationd```
