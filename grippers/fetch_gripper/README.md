# Notes

- The gripper urdf has the coordinate frame origin at the finger tip. 
- This frame has a translation offset of `delta = 0.19645` meters in the `+X` axis i.e 
  the fingertip frame is ahead by `delta`.

The older base frame in the fingertip frame `RT_oldbase_in_fingertip`

```python
# RT_oldbase_in_fingertip = [  1   0   0  -delta ]
#                           [  0   1   0    0    ]
#                           [  0   0   1    0    ]
#                           [  0   0   0    1    ]

delta = 0.19645
RT_oldbase_in_fingertip = np.eye(4)
RT_oldbase_in_fingertip[0, 3] = -delta

# get pose for oldbase give pose for fingertip link (RT_fingertip_in_world)
RT_oldbase_in_world = RT_fingertip_in_world @ RT_oldbase_in_fingertip
```

Visualization of fingertip link:

![Reference Image](../../media/fingertip_links.png)