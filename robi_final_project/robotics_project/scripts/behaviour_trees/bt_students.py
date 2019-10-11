#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# go to door until at door
		#b0 = pt.composites.Selector(
		#	name="Go to door fallback", 
		#	children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		#)

		# tuck the arm
		b0 = tuckarm()

		# lower head
		b1 = movehead("down")
		
		#pick the cube
		b2 = pick_cube()

		go1 = pt.composites.Selector(name="Main sequence", children=[counter(30, "At table?"), go("Go to table!", 0, 0.5)])

		go2 = pt.composites.Selector(name="Main sequence", children=[counter(10, "At table?"), go("Go to table!", 0.4, 0)])

		go3 = pt.composites.Selector(name="Main sequence", children=[counter(32, "At table?"), go("Go to table!", 0, 0.5)])

		go4 = pt.composites.Selector(name="Main sequence", children=[counter(11, "At table?"), go("Go to table!", 0.4, 0)])

		# go to table
		b3 = RSequence(
			name="Go to table fallback",
			children=[go1, go2 ]
		)

		# Place the cube
		
		b4 = place_cube()

		b5= cube_placed()

		b6 = RSequence(
			name="Place the cube and check with the camera if the cube is placed",
			children=[b4, b5]
		)


		# if the cube is not placed
		
		b7 = RSequence(
			name="Go back to table 1 fallback; if cube is not placed",
			children=[go3, go4 ]
		)
	
		go5 = pt.composites.Selector(name="Main sequence", children=[b6, b7])
		 

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, go5])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
