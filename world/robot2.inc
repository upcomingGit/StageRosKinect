define topurg ranger
(
	sensor
	(
		range [0 3]
		fov 270
		samples 1081
	)

	color "black"
	size [0.05 0.05 0.1]
)

define roomba2 position
(
	size [0.35 0.35 0.25]

	block
	(
		   points 16
           point[0]  [ 0.225 0.000 ]
           point[1]  [ 0.208 0.086 ]
           point[2]  [ 0.159 0.159 ]
           point[3]  [ 0.086 0.208 ]
           point[4]  [ 0.000 0.225 ]
           point[5]  [ -0.086 0.208 ]
           point[6]  [ -0.159 0.159 ]
           point[7]  [ -0.208 0.086 ]
           point[8]  [ -0.225 0.000 ]
           point[9]  [ -0.208 -0.086 ]
           point[10] [ -0.159 -0.159 ]
           point[11] [ -0.086 -0.208 ]
           point[12] [ -0.000 -0.225 ]
           point[13] [ 0.086 -0.208 ]
           point[14] [ 0.159 -0.159 ]
           point[15] [ 0.208 -0.086 ]
           z [0 10]

           color "red"
	)

	topurg(pose [0 0 0.11 0])
	color "gray50"
)
