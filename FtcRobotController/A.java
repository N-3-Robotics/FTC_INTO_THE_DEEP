val currentposition = ROBOT.LIFT.currentPosition
                if (currentposition < MaxExtention && gamepad2.right_stick_y > 0) {
                    ROBOT.LIFT.power = -gamepad2.right_stick_y.toDouble()
                } else if (currentposition > MaxExtention && gamepad2.right_stick_y < 0){
                    ROBOT.LIFT.power = 0.0
                }

            }