import pygame
import math
from queue import PriorityQueue
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
import time

WIDTH = 600 # define the width of the window
WIN = pygame.display.set_mode((WIDTH, WIDTH))
screencolor = [224, 224, 224]
WIN.fill(screencolor)
pygame.display.set_caption("Optimal Path Finding with A* Algorithm")

# color combinations
RED = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
BLUEGREEN = (64, 224, 208)

# this function is used to display the guide to run the program in few steps. 
def walk_through():
   # remove the button and label
   next_button.destroy()
   welcome_label.destroy()
   steps_label = Label(window, text="(1) - First Mark Start and Goal Point Anywhere in the grids.\n\n" +
				"(2) - Add obstacles between point using 'mouse press or the mouse drag'.\n\n" +
				"(3) - Then Hit the 'Enter Key' once you are done adding obstacles.\n\n" +
				"(Hit [ctrl + c] to clear the window)\n\n" +
				"(close this Dialogue Box to Continue)")
   # creates a new label to the GUI
   steps_label.pack()

#a new window initiated
window = Tk()
window.title("Welcome!")
window.configure(background='black')
welcome_label = Label(window, text='Welcome To Optimal Path Finding with A* Algorithm', fg='white', bg='black')
welcome_label.grid(row=5, pady=10)
welcome_label.pack()

# UI design for Button
next_button = Button(window, text="Next", bg='blue', fg='white', command=walk_through)
next_button.pack()
window.update()
mainloop()

'''
	Class with all the function that is used in the application and will be call when ever needed in the 
	application
'''
class Node:
	def __init__(self, row, column, width, total_rows):
		self.blocks = 0
		self.row = row
		self.column = column
		self.x = row * width
		self.y = column * width
		self.color = WHITE
		self.neighbours = []
		self.width = width
		self.total_rows = total_rows

	def get_position(self):
		return self.row, self.column

	def is_obstacles(self):
		return self.color == BLACK

	def reset(self):
		self.color = WHITE

	def start_position(self):
		self.color = GREEN

	def create_closed(self):
		self.color = BLUEGREEN

	def create_open(self):
		self.color = ORANGE

	def create_obstacles(self):
		self.color = BLACK

	def goal_position(self):
		self.color = RED

	def create_path(self):
		self.color = PURPLE

	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	# function to add the grid row or column to the neighbours list if the spot is not obstacles 
	def add_adjacents(self, grid):
		self.neighbours = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.column].is_obstacles(): # DOWN
			self.neighbours.append(grid[self.row + 1][self.column])

		if self.row > 0 and not grid[self.row - 1][self.column].is_obstacles(): # UP
			self.neighbours.append(grid[self.row - 1][self.column])

		if self.column < self.total_rows - 1 and not grid[self.row][self.column + 1].is_obstacles(): # RIGHT
			self.neighbours.append(grid[self.row][self.column + 1])

		if self.column > 0 and not grid[self.row][self.column - 1].is_obstacles(): # LEFT
			self.neighbours.append(grid[self.row][self.column - 1])

	def __lt__(self, other): # less than
		return False

# function to create grid
def create_grid(rows, width):
	grid = [] # empty list
	gap = width // rows
	for i in range(rows):
		grid.append([]) # 2D list [[], [], []]
		for j in range(rows):
			nodes = Node(i, j, gap, rows)
			grid[i].append(nodes)
	return grid

# create grid horizontal and vertical lines in the window
def draw_grid(win, rows, width):
	gap = width // rows
	for i in range(rows):
		# paygame.dran.line(Surface, color, start_pos, end_pos, width=1)
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap)) # horizontal
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width)) # vertical

# get position of the spot clicked in the grid (rows and columns)
def get_clicked_position(position, rows, width):
	gap = width // rows
	y, x = position

	row = y // gap
	column = x // gap

	return row, column

# draw the main grid in the window
def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()

# function to create the path
def create_path(came_from, current, draw):
	while current in came_from:
		current = came_from[current]
		current.create_path()
		draw()

# heuristic function that is used in A* serach Algorithm
def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)  # (1, 9) and (20, 30)

# A* algorithm
def a_star_algorithm(draw, grid, start, end):
	count = 0
	open_set = PriorityQueue() # variant of Queue that retrives open entries in priority order (lowest first)
	open_set.put((0, count, start)) # starts by adding the start node
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}  # inf = infinity key for spot
	g_score[start] = 0
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = h(start.get_position(), end.get_position()) # distance

	open_set_hash = {start}

	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

		current = open_set.get()[2]
		open_set_hash.remove(current)

		if current == end: # if the current node is the end node create path and end the process
			create_path(came_from, end, draw)
			end.goal_position()
			return True

		# the A* search algorithm part considering the heuristics and the actual values and deciding meanwhile if the movement should be in the neighbour or not and it represent with create_open() function
		for neighbour in current.neighbours:
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbour]:
				came_from[neighbour] = current
				g_score[neighbour] = temp_g_score
				f_score[neighbour] = temp_g_score + h(neighbour.get_position(), end.get_position())
				if neighbour not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbour], count, neighbour))
					open_set_hash.add(neighbour)
					neighbour.create_open()
		draw()

		if current != start: # if the current not is not start not create it closed
			current.create_closed()
	return False

# this main function is to check for the events while running
def main(win, width):
	ROWS = 40 # initialized the rows in the grid  
	grid = create_grid(ROWS, width) # pass ROWS and width to create_grid() function

	# initiate the start and end to None first
	start = None
	goal = None

	run = True
	while run: # While run = True
		draw(win, grid, ROWS, width)
		
		# check all the event that happens in the pygame 
		for event in pygame.event.get():
			if event.type == pygame.QUIT: # if the Window is closed then run = False and exits the window
				run = False

			if pygame.mouse.get_pressed()[0]: # LEFT - when left mouse button pressed 
				mouse_pos = pygame.mouse.get_pos()
				row, column = get_clicked_position(mouse_pos, ROWS, width)
				spot = grid[row][column]
				if not start and spot != goal: # set the start positon with start_position() function
					start = spot
					start.start_position()

				elif not goal and spot != start: # set the goal position with goal_position() function
					goal = spot
					goal.goal_position()

				elif spot != goal and spot != start: # create obstacles in the grids 
					spot.create_obstacles()

			elif pygame.mouse.get_pressed()[2]: # RIGHT - when right mouse button pressed then erase the spot
				mouse_pos = pygame.mouse.get_pos()
				row, column = get_clicked_position(mouse_pos, ROWS, width)
				spot = grid[row][column]
				spot.reset()
				if spot == start:
					start = None
				elif spot == goal:
					goal = None

			# events for keyboard keys
			if event.type == pygame.KEYDOWN:
				# if return or enter key pressed
				if event.key == pygame.K_RETURN and start and goal:
					for row in grid:
						for spot in row:
							spot.add_adjacents(grid)

					start_time = time.process_time()
					print("Start Time:", start_time)
					
					# call the a* algorithm function and draw function
					a_star_algorithm(lambda: draw(win, grid, ROWS, width), grid, start, goal)

					end_time = time.process_time()
					print("End Time:", end_time)
					total_time = round(end_time - start_time, 2)
					print(f"Time Taken: {total_time}")

					# this part of the code, will display the dialogue box with the total time taken to find the path with the help of the tkinter function
					Tk().wm_withdraw()
					result = messagebox.showinfo('Path Found', ('The time taken to find the path is '+ str(total_time)+' seconds.'))

				# if ctrl + c pressed then clear all the spots i.e. start, end and obstacles
				if event.key == pygame.K_c:
					start = None
					goal = None
					grid = create_grid(ROWS, width)

	pygame.quit()

# run the main function with pygame WIN and WINDOW = 600
main(WIN, WIDTH)