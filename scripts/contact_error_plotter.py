#!/usr/bin/env python3
import rclpy.duration
from rclpy.node import Node
import rclpy
import tkinter as tk
from haptiquad_plot.libs import PlotContainer
from ament_index_python.packages import get_package_share_directory
from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, TransformStamped, Quaternion
import numpy as np
from threading import Thread
from PIL import Image, ImageTk
from tkinter import ttk
from copy import deepcopy
import tf2_ros
import quaternion as q
import tkinter.font as tkFont
import os
from datetime import datetime
import matplotlib.pyplot as plt

names = ["error", "norm", "comp"]


class ContactError(Node, tk.Tk):

	def __init__(self):
		Node.__init__(self, "ContactError")
		tk.Tk.__init__(self)

		self.protocol('WM_DELETE_WINDOW', self.on_destroy)
		self.geometry("1280x720")
		tk.Tk.wm_title(self, "ContactError")

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		try:
			theme_path = f"{get_package_share_directory('haptiquad_plot')}/theme/azure.tcl"
			self.tk.call("source", theme_path)
			self.tk.call("set_theme", "light")
		except:
			pass

		self.previous_size = (0, 0)

		self.rate = self.create_rate(60)

		self.plot = PlotContainer(self, f"Contact Point Error")

		pause_icon_path = f"{get_package_share_directory('haptiquad_contacts')}/scripts/res/pause_icon.png"
		save_icon_path = f"{get_package_share_directory('haptiquad_contacts')}/scripts/res/save_icon.png"

		self.pause_icon = ImageTk.PhotoImage(Image.open(pause_icon_path))
		self.save_icon = ImageTk.PhotoImage(Image.open(save_icon_path).resize((32,32)))


		self.pause = tk.BooleanVar()
		self.pause.set(False)
		self.pause_button = ttk.Checkbutton(self, style='Toggle.TButton', image=self.pause_icon, variable=self.pause)
		self.pause_button.grid(column=0, row=0, pady=10, sticky="w", padx=(20,5))

		self.save_button = ttk.Button(self, image=self.save_icon, command=self.save_plots)
		self.save_button.grid(column=1, row=0, pady=10, sticky="w", padx=5)

		self.plot.grid(row=1, column=0, sticky="we", columnspan = 100)
		self.plot.adjust_plots(0.1, 0.19, 0.938, 0.95, 0.2, 0.165)
		self.plot.autoscroll = True
		self.plot.autoscale = True
		self.plot.x_range = 10.0
		self.limit = 1000


		self.to_plot = {}
		self.to_plot["error"] = np.empty((3,0))
		self.to_plot["comp"] = np.empty((6,0))
		self.to_plot["norm"] = np.empty((1,0))
		self.to_plot["time"] = np.empty(0)


		self.mode = tk.IntVar()
		self.mode.set(0)
		self.prev_mode = self.mode.get()

		ttk.Separator(self, orient=tk.VERTICAL).grid(row=0, column=2, sticky="ns", pady=5, padx = 5)
		font = tkFont.Font(family='Segoe Ui', size=16)

		ttk.Label(self, text="Mode :", font=font).grid(row = 0, column=3, sticky="ns", pady = 5, padx = (0, 0))
		self.show_err_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Error", variable=self.mode, width=10, value=0)
		self.show_err_butt.grid(row = 0, column=4, padx = (5,5), pady = 10, sticky="w")

		self.show_norm_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Norm Error", variable=self.mode, width=10, value=1)
		self.show_norm_butt.grid(row = 0, column=5, padx = (5,5), pady = 10, sticky="w")

		self.show_both_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Comparison", variable=self.mode, width=10, value=2)
		self.show_both_butt.grid(row = 0, column=6, padx = (5,5), pady = 10, sticky="w")

		

		self.start_time = None
		self.frozen_data = None

		self.estimated_subscriber = Subscriber(self, Marker, "/visualization/estimated_contact")
		self.gt_subscriber = Subscriber(self, Marker, "/visualization/mujoco_contacts")
		self.subs = [self.gt_subscriber, self.estimated_subscriber]
		self.sync = ApproximateTimeSynchronizer(self.subs, queue_size=10, slop=0.1)
		self.sync.registerCallback(self.callback)

		self.bind("<Configure>", self.on_resize) 
		self.on_resize(None)

		self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
		self.spin_thread.start()




	def on_destroy(self):

		self.destroy()
		self.destroy_node()
		rclpy.shutdown()
		self.spin_thread.join()


	def run(self):

		while rclpy.ok():

			self.update_plots()
			self.update()
			self.update_idletasks()
			self.rate.sleep()


	def on_resize(self, event):

		#Update to get the current values
		self.update()
		curr_size = (self.winfo_width(), self.winfo_height())

		if self.previous_size != curr_size:

			height = self.winfo_height() -20
			width = self.winfo_width() -20

			self.plot.canvas.get_tk_widget().config(width=width, height=height)
			self.plot.fast_update()

		self.previous_size = curr_size

	def transform_pose(self, pose, transform):
		# Extract translation and rotation from the transform
		t = transform.transform.translation
		q_t = transform.transform.rotation
		T_trans = np.array([t.x, t.y, t.z])
		Q_trans = q.quaternion(q_t.w, q_t.x, q_t.y, q_t.z)

		# Extract pose position and orientation
		p = pose.position
		q_p = pose.orientation
		P_trans = np.array([p.x, p.y, p.z])
		Q_pose = q.quaternion(q_p.w, q_p.x, q_p.y, q_p.z)

		# Rotate the position
		P_rotated = q.as_rotation_matrix(Q_trans) @ P_trans + T_trans

		# Rotate the orientation
		Q_result = Q_trans * Q_pose

		# Compose new Pose message
		new_pose = Pose()
		new_pose.position.x = P_rotated[0]
		new_pose.position.y = P_rotated[1]
		new_pose.position.z = P_rotated[2]
		new_pose.orientation = Quaternion(
			x=Q_result.x,
			y=Q_result.y,
			z=Q_result.z,
			w=Q_result.w
		)

		return new_pose


	def callback(self, *msgs):

		[gt, est] = msgs

		t = gt.header.stamp.sec + 1e-9 * gt.header.stamp.nanosec
		if self.start_time == None:
			self.start_time = t		
		normalized_time = t-self.start_time

		self.to_plot["time"] = np.append(self.to_plot["time"], normalized_time)

		try:
			# Get the transform from source_frame to target_frame
			transform: TransformStamped = self.tf_buffer.lookup_transform(
				"base",
				"world",
				rclpy.time.Time(),
				rclpy.duration.Duration(seconds=0.1)
			)

			# Apply the transform manually
			gt_p = self.transform_pose(gt.pose, transform)

		except Exception as e:
			self.get_logger().warn(f'Failed to transform: {e}')
			return



		gt_position = np.array([
			gt_p.position.x,
			gt_p.position.y,
			gt_p.position.z,
		]).reshape((3,1))

		est_position = np.array([
			est.pose.position.x,
			est.pose.position.y,
			est.pose.position.z,
		]).reshape((3,1))

		comparison = np.array([
			gt_p.position.x,
			est.pose.position.x,
			gt_p.position.y,
			est.pose.position.y,
			gt_p.position.z,
			est.pose.position.z
		]).reshape((6,1))

		norm_err = np.linalg.norm(gt_position - est_position)
		err = gt_position - est_position
		self.to_plot["error"] = np.append(self.to_plot["error"], err, axis=1)
		self.to_plot["norm"] = np.concatenate((self.to_plot["norm"], np.array([[norm_err]])), axis=1)
		self.to_plot["comp"] = np.append(self.to_plot["comp"], comparison, axis=1)

		if self.to_plot["error"].shape[1] >= self.limit:
			self.to_plot["error"] = self.to_plot["error"][:,1:]
			self.to_plot["norm"] = self.to_plot["norm"][:, 1:]
			self.to_plot["comp"] = self.to_plot["comp"][:, 1:]
			self.to_plot["time"] = self.to_plot["time"][1:]

	
	def process_mode(self, mode):

		color = None		
		style = None
		xlabel = "Time [s]"
		pause = self.pause.get()
		x_values = self.to_plot["time"] if not pause else self.frozen_data["time"]

		if mode == 0:
			y_values = self.to_plot["error"] if not pause else self.frozen_data["error"]
			title = "Contact Point Error"
			ylabel = "Meters [m]"
			legend = ["X", "Y", "Z"]

		if mode == 1:
			y_values = self.to_plot["norm"] if not pause else self.frozen_data["norm"]
			title = "Contact Point Norm Error"
			ylabel = "Meters [m]"
			legend = ["Norm Error"]

		if mode == 2:
			y_values = self.to_plot["comp"] if not pause else self.frozen_data["comp"]
			title = "Contact Point Comparison"
			ylabel = "Meters [m]"
			legend = ["GT X", "EST X", "GT Y", "EST Y", "GT Z", "EST Z"]
			style = ['-', '--', '-', '--', '-', '--']
			color = ['#1f77b4', '#14425f', '#ff7f0e', '#b0421e', '#2ca02c', '#4e781e']



		return y_values, x_values, legend, title, xlabel, ylabel, color, style


	def update_plots(self):

		mode = self.mode.get()

		if self.prev_mode != self.mode.get():
			self.prev_mode = self.mode.get()
			self.plot.clear()
		
		if self.pause.get() and type(self.frozen_data) == type(None):

			self.frozen_data = deepcopy(self.to_plot)

		elif not self.pause.get() and type(self.frozen_data) == dict:

			del self.frozen_data
			self.frozen_data = None

		y_values, x_values, legend, title, xlabel, ylabel, color, style = self.process_mode(mode)

		if not y_values.shape[1] > 0:
			return
		if not x_values.shape[0] == y_values.shape[1]:
			return
		
		self.plot.update_plot(y_values, legend, title=title, xlabel= xlabel, ylabel=ylabel, time = x_values, color=color, style=style)


	def save_plots(self):
		
		path = tk.filedialog.askdirectory(title="Choose a Directory")
		now = datetime.now()
		time_str = now.strftime("%Y-%m-%d_%H-%M-%S")
		new_path = os.path.join(path, f"contacts_output_{time_str}")

		os.makedirs(new_path)   

		for m in range(3):

			y_values, x_values, legend, title, xlabel, ylabel, color, style = self.process_mode(m)
			fig, ax = plt.subplots(constrained_layout=True)
			fig.set_size_inches(9.4, 4.8)


			ax.grid(True)
			ax.set_title(self.title if title==None else title, fontsize=16)  
			for j in range(y_values.shape[0]):
				if not type(style) == type(None):
					s = style[j]
					c = color[j]
				else:
					s = style
					c = color

				ax.plot(x_values, y_values[j], label=legend[j], color=c, linestyle=s, linewidth=3)
				ax.legend(loc="upper right")                
				ax.set_xlabel("" if xlabel == None else xlabel, fontsize =14)
				ax.set_ylabel("" if ylabel == None else ylabel, fontsize = 14)

				ax.relim()
				ax.autoscale()
				ax.set_xlim(x_values[-1]-10.2, x_values[-1])
				filename = os.path.join(new_path, f"{names[m]}.png")
				self.get_logger().info(f'Saving image: {filename}')
				fig.savefig(filename, dpi=200)
				plt.close(fig)  
		
		self.save_stats(new_path)


	def save_stats(self, filepath):

		if self.frozen_data == None:
			return

		with open(os.path.join(filepath, 'contact_stats.txt'), "w") as f:
			f.write("Contact stats:\n")
	
			avg_norm = np.mean(self.frozen_data['norm'])
			avg_err_x = np.mean(self.frozen_data['error'][0,:])
			avg_err_y = np.mean(self.frozen_data['error'][1,:])
			avg_err_z = np.mean(self.frozen_data['error'][2,:])
			min_norm = np.min(self.frozen_data['norm'])
			max_norm = np.max(self.frozen_data['norm'])

			f.write('   Norm:\n')
			f.write(f'      - Avg value: {avg_norm} m\n')
			f.write(f'      - Min value: {min_norm} m\n')
			f.write(f'      - Max value: {max_norm} m\n')
			f.write('   Error:\n')
			f.write('       -X:\n')
			f.write(f'          - Avg value: {avg_err_x} m\n')
			f.write('       -Y:\n')
			f.write(f'          - Avg value: {avg_err_y} m\n')
			f.write('       -Z:\n')
			f.write(f'          - Avg value: {avg_err_z} m\n')
  


		

def main(args = None):

	rclpy.init(args=args)
	node = ContactError()
	node.run()



if __name__ == '__main__':
	main()
