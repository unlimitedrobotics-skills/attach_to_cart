# Ra-Ya Skill - Attach to Cart

Intro: 
This skill is used when gary required to attach to cart.
Attach to cart skill uses approach to tag skill inside the logic

Required hardware:
Gary mast have Cart gripper with one of the two:
2 IR sensors as part of the gripper and 1 skirt integrated back SRF
OR
2 SRF sensors as part of the gripper

Basic logic:
1. approach to tag
2. rotate 180 degrees
3. state machine which runs on three posible states:
    3.1 linear velocity backwards
    3.2 adjust angle
    3.3 attach to cart
4. after attaching gary run cart attach verification with moving forward and
    riding back SRF values to identify if the cart is connected

All constant parameters can be changed inside constants.py file.

required Execute parameters:
1. april tag size --> -s
2. april tag Identifiers --> i

run command line for example:
``` bash
rayasdk run -s 0.06 -i 1 2
```


