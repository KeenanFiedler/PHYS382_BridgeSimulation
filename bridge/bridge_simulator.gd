extends Node2D

# ============================================================================
# BRIDGE FINITE ELEMENT METHOD (FEM) SIMULATOR
# ============================================================================
# A 2D structural analysis simulator using finite element methods to model
# bridge behavior under load. Supports multiple materials, dynamic physics,
# stress analysis, and failure simulation.
# ============================================================================

# ============================================================================
# MATERIAL TYPES AND PROPERTIES
# ============================================================================
enum MaterialType {
	WOOD,   # Lower strength, lower modulus
	STEEL,  # High strength, high modulus, small cross-section
	ROAD    # Medium properties, larger cross-section for deck
}

# Material properties database
# - density: Mass per unit length (kg/pixel-unit)
# - youngs_modulus: Elastic modulus (Pa) - resistance to deformation
# - cross_section_area: Cross-sectional area (pixel-units²)
# - yield_strength: Stress at which permanent deformation begins (Pa)
# - ultimate_strength: Stress at which material fails/breaks (Pa)
# - color: Visual representation color
# - name: Display name
const MATERIAL_PROPERTIES := {
	MaterialType.WOOD: {
		"density": 4.39, 
		"youngs_modulus": 1.0e9,
		"cross_section_area": 5.7,
		"yield_strength": 5.0e6,
		"ultimate_strength": 1.05e7,
		"color": Color(0.65, 0.45, 0.2),
		"name": "Wood"
	},
	MaterialType.ROAD: {
		"density": 25.2,
		"youngs_modulus": 2.5e9,
		"cross_section_area": 10.5,
		"yield_strength": 3.0e6,
		"ultimate_strength": 4.5e6,
		"color": Color(0.231, 0.278, 0.278),
		"name": "Road"
	},
	MaterialType.STEEL: {
		"density": 15.1,
		"youngs_modulus": 2.0e10,
		"cross_section_area": 2.785,
		"yield_strength": 2.5e7,
		"ultimate_strength": 4.0e7,
		"color": Color(0.688,0.766,0.867),
		"name": "Steel"
	}
}

# ============================================================================
# ENVIRONMENT CONSTANTS
# ============================================================================

const GROUND_Y = 450                # Y-coordinate of ground level (pixels)
const WATER_Y = 500                 # Y-coordinate of water surface (pixels)
const BRIDGE_SUPPORT_LEFT_X = 200   # Left bridge support X-position (pixels)
const BRIDGE_SUPPORT_RIGHT_X = 1080 # Right bridge support X-position (pixels)

# ============================================================================
# PHYSICS SIMULATION VARIABLES
# ============================================================================
var impulse_test_mode: bool = false
var impulse_test_node: FEMNode = null
var impulse_magnitude: float = 50000.0  # Impulse force magnitude
var nodes: Array[FEMNode] = []      # All structural nodes in the bridge
var elements: Array[FEMElement] = [] # All structural elements (beams) connecting nodes
var weights: Array[LoadWeight] = []  # Applied point loads on nodes

# Gravitational acceleration vector (pixels/s²)
# Standard Earth gravity scaled to simulation units
var gravity: Vector2 = Vector2(0, 98.1)

# Rayleigh damping coefficients
# These damp oscillations to simulate energy dissipation in real structures
var alpha_damping: float = 0.1986    # Mass-proportional damping (damps low frequencies)
var beta_damping: float = 0.00011533   # Stiffness-proportional damping (damps high frequencies)

# Time integration parameters
var time_step: float = 0.0008333333    # Physics timestep (1/1200 second)
var sub_steps: int = 20                # Number of substeps per frame for stability
var simulation_running: bool = false   # Whether physics is currently running
var current_bridge_index: int = 0      # Index of currently loaded preset bridge

# ============================================================================
# STRESS HISTORY TRACKING
# ============================================================================
# Stores stress data over time for frequency analysis and structural study

var stress_history: Array = []              # 2D array: [timestep][element] stress values
var recording_stress: bool = false          # Whether currently recording stress data
var stress_recording_duration: float = 10.0 # Duration to record (seconds)
var stress_recording_elapsed: float = 0.0   # Time elapsed during recording (seconds)
var stress_recording_interval: float = 0.0  # Time between samples (set to time_step * sub_steps)

# ============================================================================
# USER INTERFACE STATE
# ============================================================================

var selected_material: MaterialType = MaterialType.STEEL # Current material for new beams
var placement_mode: String = "node"                      # Current tool: "node", "beam", or "weight"
var first_node: FEMNode = null                           # First node selected when placing beams
var mouse_pos: Vector2 = Vector2.ZERO                    # Current mouse position (pixels)
var snapped_mouse_pos: Vector2 = Vector2.ZERO            # Mouse position snapped to grid
var selected_weight_mass: float = 10000.0                # Mass for next weight placement (kg)
const GRID_SIZE: int = 10                                # Grid snap size (pixels)

# ============================================================================
# NODE CLASS
# ============================================================================
# Represents a structural connection point in the bridge
# Nodes store position, velocity, and can be fixed (pinned) to ground

class FEMNode:
	var pos: Vector2              # Current position (pixels)
	var original_pos: Vector2     # Initial position for reset (pixels)
	var velocity: Vector2 = Vector2.ZERO  # Current velocity (pixels/s)
	var force: Vector2 = Vector2.ZERO     # Net force acting on node (N)
	var mass: float = 0.0         # Mass from connected elements (kg)
	var fixed: bool = false       # Whether node is pinned (cannot move)
	var applied_mass: float = 0.0 # Additional mass from applied weights (kg)
	
	func _init(p: Vector2, is_fixed: bool = false):
		pos = p
		original_pos = p
		fixed = is_fixed

# ============================================================================
# ELEMENT CLASS
# ============================================================================
# Represents a structural beam connecting two nodes
# Elements have material properties and can yield or break under stress

class FEMElement:
	var node_a: FEMNode           # First endpoint node
	var node_b: FEMNode           # Second endpoint node
	var material_type: MaterialType # Material this element is made of
	var original_length: float    # Rest length (pixels)
	var broken: bool = false      # Whether element has failed (broken)
	var yielded: bool = false     # Whether element has yielded (permanent deformation)
	var element_mass: float = 0.0 # Total mass of this element (kg)
	var stiffness: float = 0.0    # Axial stiffness k = EA/L (N/pixel)
	
	func _init(a: FEMNode, b: FEMNode, mat: MaterialType):
		node_a = a
		node_b = b
		material_type = mat
		original_length = a.pos.distance_to(b.pos)
		
		var props = MATERIAL_PROPERTIES[mat]
		element_mass = props["density"] * original_length
		
		# Distribute half of element mass to each endpoint node
		# This creates a consistent mass matrix for the structure
		a.mass += element_mass / 2.0
		b.mass += element_mass / 2.0
		
		# Calculate axial stiffness: k = (E * A) / L
		# Higher stiffness resists deformation more strongly
		stiffness = (props["youngs_modulus"] * props["cross_section_area"]) / original_length
	
	# Calculate current axial stress in the element (Pa)
	# Stress = E * strain, where strain = (L - L0) / L0
	# Positive stress = tension, negative stress = compression
	func calculate_stress() -> float:
		var current_length = node_a.pos.distance_to(node_b.pos)
		var strain = (current_length - original_length) / original_length
		var props = MATERIAL_PROPERTIES[material_type]
		var E = props["youngs_modulus"]
		return E * strain
	
	# Calculate current strain (dimensionless)
	# Strain = (L - L0) / L0
	# Positive = elongation, negative = compression
	func calculate_strain() -> float:
		var current_length = node_a.pos.distance_to(node_b.pos)
		return (current_length - original_length) / original_length
	
	# Calculate current axial force in the element (N)
	# Force = stress * cross_sectional_area
	func calculate_force() -> float:
		var stress = calculate_stress()
		var props = MATERIAL_PROPERTIES[material_type]
		var area = props["cross_section_area"]
		return stress * area
	
	# Check if element has yielded or broken
	# Updates yielded and broken flags based on current stress
	func check_failure():
		var stress = abs(calculate_stress())
		var props = MATERIAL_PROPERTIES[material_type]
		if stress > props["yield_strength"]:
			yielded = true
		if stress > props["ultimate_strength"]:
			broken = true
	
	# Get stress ratio (0.0 to 1.0+) relative to ultimate strength
	# Used for visual color coding of stress levels
	func get_stress_ratio() -> float:
		var stress = abs(calculate_stress())
		var props = MATERIAL_PROPERTIES[material_type]
		return stress / props["ultimate_strength"]

# ============================================================================
# LOAD WEIGHT CLASS
# ============================================================================
# Represents an applied point load on a node

class LoadWeight:
	var node: FEMNode  # Node this weight is attached to
	var mass: float    # Mass of the weight (kg)
	var color: Color   # Visual color for display
	
	func _init(n: FEMNode, m: float):
		node = n
		mass = m
		color = Color(0.8, 0.2, 0.2)

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready():
	_load_bridge(0)

# ============================================================================
# BRIDGE PRESETS
# ============================================================================
# Three pre-built bridge designs for testing

func _load_bridge(index: int):
	current_bridge_index = index
	_clear_all()
	match index:
		0: _create_truss_bridge()
		1: _create_arch_bridge()
		2: _create_simple_beam()

# Create a Warren truss bridge design
# Features triangular truss pattern for efficient load distribution
func _create_truss_bridge():
	# Bottom chord nodes (road level)
	var n1 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X, GROUND_Y), true)
	nodes.append(n1)
	
	var n2 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+100, GROUND_Y))
	var n3 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+200, GROUND_Y))
	var n4 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+300, GROUND_Y))
	var n5 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+400, GROUND_Y))
	var n6 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+500, GROUND_Y))
	var n7 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+600, GROUND_Y))
	var n8 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+700, GROUND_Y))
	var n9 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X+800, GROUND_Y))
	nodes.append_array([n2, n3, n4, n5, n6, n7, n8, n9])
	
	var n10 = FEMNode.new(Vector2(BRIDGE_SUPPORT_RIGHT_X, GROUND_Y), true)
	nodes.append(n10)
	
	# Top chord nodes (upper truss level)
	var n11 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 100, GROUND_Y - 150))
	var n12 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 200, GROUND_Y - 150))
	var n13 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 300, GROUND_Y - 150))
	var n14 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 400, GROUND_Y - 150))
	var n15 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 500, GROUND_Y - 150))
	var n16 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 600, GROUND_Y - 150))
	var n17 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 700, GROUND_Y - 150))
	var n18 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X + 800, GROUND_Y - 150))
	nodes.append_array([n11, n12, n13, n14, n15, n16, n17, n18])
	
	# Bottom road deck elements
	for i in range(9):
		elements.append(FEMElement.new(nodes[i], nodes[i+1], MaterialType.ROAD))
	
	# Top truss chord elements
	for i in range(10,17):
		elements.append(FEMElement.new(nodes[i], nodes[i+1], MaterialType.STEEL))
	
	# Vertical support members
	for i in range(1,9):
		elements.append(FEMElement.new(nodes[i], nodes[i+9], MaterialType.STEEL))
	
	# Diagonal members (creates triangular truss pattern)
	for i in range(5):
		elements.append(FEMElement.new(nodes[i], nodes[i+10], MaterialType.STEEL))
	for i in range(5,10):
		elements.append(FEMElement.new(nodes[i], nodes[i+8], MaterialType.STEEL))

# Create an arch bridge design
# Load is transferred through compression in the arch
func _create_arch_bridge():
	# Fixed support nodes
	var n1 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X, GROUND_Y), true)
	var n2 = FEMNode.new(Vector2(BRIDGE_SUPPORT_RIGHT_X, GROUND_Y), true)
	nodes.append_array([n1, n2])
	
	# Create parabolic arch nodes
	var arch_nodes = []
	for i in range(9):
		var t = (i + 1) / 10.0  # Parametric position along span (0.1 to 0.9)
		var x = lerp(BRIDGE_SUPPORT_LEFT_X, BRIDGE_SUPPORT_RIGHT_X, t)
		var arch_height = sin(t * PI) * 200  # Sinusoidal arch shape
		var arch_node = FEMNode.new(Vector2(x, GROUND_Y - arch_height))
		nodes.append(arch_node)
		arch_nodes.append(arch_node)
	
	# Connect arch segments
	var prev = n1
	for an in arch_nodes:
		elements.append(FEMElement.new(prev, an, MaterialType.STEEL))
		prev = an
	elements.append(FEMElement.new(prev, n2, MaterialType.STEEL))
	
	# Create deck nodes at road level
	for an in arch_nodes:
		var deck_node = FEMNode.new(Vector2(an.pos.x, GROUND_Y))
		nodes.append(deck_node)
		# Vertical hangers connect deck to arch
		elements.append(FEMElement.new(an, deck_node, MaterialType.STEEL))
	
	# Connect road deck segments
	var deck_start = n1
	for i in range(nodes.size() - 9, nodes.size()):
		elements.append(FEMElement.new(deck_start, nodes[i], MaterialType.ROAD))
		deck_start = nodes[i]
	elements.append(FEMElement.new(deck_start, n2, MaterialType.ROAD))

# Create a simple beam bridge
# Most basic design - just a continuous deck between supports
func _create_simple_beam():
	# Fixed support nodes at ends
	var n1 = FEMNode.new(Vector2(BRIDGE_SUPPORT_LEFT_X, GROUND_Y), true)
	var n2 = FEMNode.new(Vector2(BRIDGE_SUPPORT_RIGHT_X, GROUND_Y), true)
	nodes.append_array([n1, n2])
	
	# Create intermediate deck nodes
	var prev = n1
	for i in range(7):
		var x = BRIDGE_SUPPORT_LEFT_X + 110 * (i + 1)
		var node = FEMNode.new(Vector2(x, GROUND_Y))
		nodes.append(node)
		elements.append(FEMElement.new(prev, node, MaterialType.ROAD))
		prev = node
	elements.append(FEMElement.new(prev, n2, MaterialType.ROAD))

# ============================================================================
# PHYSICS SIMULATION
# ============================================================================

func _physics_process(delta):
	if simulation_running:
		for _i in range(sub_steps):
			_simulate_step()
			
			if recording_stress:
				stress_recording_elapsed += time_step
				
				# Record displacement instead of stress for impulse test
				var current_data = []
				if impulse_test_mode and impulse_test_node != null:
					# Record Y-displacement of impulse node
					current_data.append(impulse_test_node.pos.y - impulse_test_node.original_pos.y)
				else:
					# Normal stress recording
					for element in elements:
						current_data.append(element.calculate_stress())
				
				stress_history.append(current_data)
				
				if stress_recording_elapsed >= stress_recording_duration:
					recording_stress = false
					impulse_test_mode = false
					_export_impulse_response()
					print("Impulse test complete! Check displacement FFT for natural frequencies.")


# Single physics timestep using semi-implicit Euler integration
# Order: compute forces → update velocities → update positions → check failures
func _simulate_step():
	var dt = time_step
	
	# ========================================
	# STEP 1: Reset and apply gravity forces
	# ========================================
	for node in nodes:
		if not node.fixed:
			var total_mass = node.mass + node.applied_mass
			node.force = gravity * total_mass  # F = mg
	
	# ========================================
	# STEP 2: Calculate element forces
	# ========================================
	for element in elements:
		if element.broken:
			continue  # Broken elements don't transmit force
		
		# Direction vector from node_a to node_b
		var delta = element.node_b.pos - element.node_a.pos
		var force_dir = delta.normalized()
		
		# Elastic force from element deformation
		# Hooke's law: F = k * (L - L0), or equivalently F = stress * area
		var force_magnitude = element.calculate_force()
		var force = force_dir * force_magnitude
		
		# Apply equal and opposite forces to endpoints (Newton's 3rd law)
		element.node_a.force += force
		element.node_b.force -= force
		
		# Stiffness-proportional damping (β damping)
		# Damps relative velocity between endpoints to reduce high-frequency oscillations
		var relative_velocity = element.node_b.velocity - element.node_a.velocity
		var velocity_along_element = relative_velocity.dot(force_dir)
		
		# Damping force proportional to stiffness and relative velocity
		# F_d = β * k * v_relative
		var damping_force_magnitude = beta_damping * element.stiffness * velocity_along_element
		var damping_force = force_dir * damping_force_magnitude
		
		element.node_a.force += damping_force
		element.node_b.force -= damping_force
	
	# ========================================
	# STEP 3: Update velocities and positions
	# ========================================
	for node in nodes:
		if node.fixed:
			continue  # Fixed nodes don't move
		
		var total_mass = node.mass + node.applied_mass
		
		# Mass-proportional damping (α damping)
		# Damps overall velocity to simulate structural damping
		# F_d = -α * m * v 
		node.force -= alpha_damping * total_mass * node.velocity
		
		# Semi-implicit Euler integration
		# More stable than explicit Euler for spring systems
		var acceleration = node.force / total_mass  # a = F/m
		node.velocity += acceleration * dt          # v(t+dt) = v(t) + a*dt
		node.pos += node.velocity * dt              # x(t+dt) = x(t) + v(t+dt)*dt
	
	# ========================================
	# STEP 4: Check for element failures
	# ========================================
	for element in elements:
		element.check_failure()

# ============================================================================
# USER INTERFACE - PROCESSING
# ============================================================================

func _process(_delta):
	mouse_pos = get_global_mouse_position()
	snapped_mouse_pos = _snap_to_grid(mouse_pos)
	queue_redraw()  # Request visual update

# Snap position to grid for easier alignment
func _snap_to_grid(pos: Vector2) -> Vector2:
	return Vector2(
		round(pos.x / GRID_SIZE) * GRID_SIZE,
		round(pos.y / GRID_SIZE) * GRID_SIZE
	)

# ============================================================================
# RENDERING
# ============================================================================

func _draw():
	# ========================================
	# Draw sky gradient background
	# ========================================
	var sky_top = Color(0.4, 0.6, 0.9)
	var sky_bottom = Color(0.6, 0.8, 1.0)
	for i in range(int(WATER_Y)):
		var t = float(i) / WATER_Y
		var color = sky_top.lerp(sky_bottom, t)
		draw_line(Vector2(0, i), Vector2(1280, i), color, 1.0)
	
	# ========================================
	# Draw grid for alignment reference
	# ========================================
	var grid_color = Color(1, 1, 1, 0.15)
	for x in range(0, 1280, GRID_SIZE):
		draw_line(Vector2(x, 0), Vector2(x, 720), grid_color, 1.0)
	for y in range(0, 720, GRID_SIZE):
		draw_line(Vector2(0, y), Vector2(1280, y), grid_color, 1.0)
	
	# ========================================
	# Draw water
	# ========================================
	var water_color = Color(0.2, 0.4, 0.7, 0.8)
	draw_rect(Rect2(0, WATER_Y, 1280, 720 - WATER_Y), water_color)
	
	# ========================================
	# Draw ground/earth
	# ========================================
	var ground_color = Color(0.3, 0.5, 0.2)
	draw_rect(Rect2(0, GROUND_Y, BRIDGE_SUPPORT_LEFT_X, 720-GROUND_Y), ground_color)
	draw_rect(Rect2(BRIDGE_SUPPORT_RIGHT_X, GROUND_Y, BRIDGE_SUPPORT_LEFT_X, 720-GROUND_Y), ground_color)
	
	# ========================================
	# Draw bridge supports/abutments
	# ========================================
	draw_rect(Rect2(BRIDGE_SUPPORT_LEFT_X - 10, GROUND_Y, 20, 10), Color(0.4, 0.4, 0.4))
	draw_rect(Rect2(BRIDGE_SUPPORT_RIGHT_X - 10, GROUND_Y, 20, 10), Color(0.4, 0.4, 0.4))
	
	# ========================================
	# Draw elements with stress-based coloring
	# ========================================
	for element in elements:
		if element.broken:
			# Broken elements shown in red
			draw_line(element.node_a.pos, element.node_b.pos, Color(1.0, 0.0, 0.0), 4.0)
		else:
			# Color interpolation based on stress ratio
			# 0% stress = material color, 100% stress = red
			var stress_ratio = element.get_stress_ratio()
			var props = MATERIAL_PROPERTIES[element.material_type]
			var base_color = props["color"]
			var clamped_stress = clamp(stress_ratio, 0.0, 1.0)
			var stress_color = Color(1.0, 0.0, 0.0)
			var color = base_color.lerp(stress_color, clamped_stress)
			var thickness = 5.0 if element.material_type == MaterialType.ROAD else 3.0
			draw_line(element.node_a.pos, element.node_b.pos, color, thickness)
	
	# ========================================
	# Draw nodes
	# ========================================
	for node in nodes:
		var color = Color.BLUE if node.fixed else Color.WHITE
		var radius = 5.0
		draw_circle(node.pos, radius, color)
		draw_circle(node.pos, radius, Color.BLACK, false, 1.5)
	
	# ========================================
	# Draw applied weights
	# ========================================
	for weight in weights:
		var weight_pos = weight.node.pos
		var size = 15
		draw_rect(Rect2(weight_pos.x - size/2, weight_pos.y - size/2, size, size), weight.color)
		draw_rect(Rect2(weight_pos.x - size/2, weight_pos.y - size/2, size, size), Color.BLACK, false, 2.0)
		draw_string(ThemeDB.fallback_font, weight_pos + Vector2(-15, -size/2 - 5), str(int(weight.mass)) + "kg", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color.WHITE)
	
	# ========================================
	# Draw placement preview/cursor
	# ========================================
	if placement_mode == "beam" and first_node != null:
		# Show line preview when placing beam
		var props = MATERIAL_PROPERTIES[selected_material]
		draw_line(first_node.pos, snapped_mouse_pos, props["color"].lightened(0.3), 2.0)
	elif placement_mode == "weight":
		# Highlight nearest node for weight placement
		var nearest = _find_nearest_node(snapped_mouse_pos, 30.0)
		if nearest != null:
			draw_circle(nearest.pos, 20, Color(1, 0, 0, 0.3))
	elif placement_mode == "node":
		# Show cursor for node placement
		draw_circle(snapped_mouse_pos, 7, Color(1, 1, 1, 0.5))
		draw_circle(snapped_mouse_pos, 7, Color(0, 0, 0, 0.5), false, 2.0)
	
	# Draw UI overlay
	_draw_ui()

# Draw user interface overlay with controls and status
func _draw_ui():
	# Semi-transparent background panel
	draw_rect(Rect2(5, 5, 460, 290), Color(0, 0, 0, 0.7))
	
	var y = 25
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "Bridge Simulator", HORIZONTAL_ALIGNMENT_LEFT, -1, 18, Color.WHITE)
	y += 28
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "[SPACE] Toggle simulation: " + ("Running" if simulation_running else "Stopped"), HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.YELLOW)
	y += 23
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "[N] Place nodes | [B] Place beams | [W] Place weights", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.WHITE)
	y += 23
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "[1] Steel | [2] Wood | [3] Road", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.WHITE)
	y += 23
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "[R] Reset | [C] Clear | [F] Toggle fixed node", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.WHITE)
	y += 23
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "[+/-] Weight: " + str(int(selected_weight_mass)) + "kg", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.CYAN)
	y += 23
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "[S] Start stress recording (10s)", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.WHITE)
	y += 23
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "[I] Truss | [O] Arch | [P] Simple Beam", HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.LIGHT_GRAY)
	y += 23
	
	var props = MATERIAL_PROPERTIES[selected_material]
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "Material: " + props["name"], HORIZONTAL_ALIGNMENT_LEFT, -1, 15, props["color"])
	y += 23
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "Mode: " + placement_mode, HORIZONTAL_ALIGNMENT_LEFT, -1, 15, Color.CYAN)
	y += 23
	
	# Show recording progress if active
	if recording_stress:
		draw_string(ThemeDB.fallback_font, Vector2(10, y), "Recording: %.1fs / %.1fs" % [stress_recording_elapsed, stress_recording_duration], HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color.RED)
		y += 23
	
	# Calculate and display structural statistics
	var total_weight = 0.0
	var broken_count = 0
	var yielded_count = 0
	
	for element in elements:
		total_weight += element.element_mass
		if element.broken:
			broken_count += 1
		elif element.yielded:
			yielded_count += 1
	
	for weight in weights:
		total_weight += weight.mass
	
	draw_string(ThemeDB.fallback_font, Vector2(10, y), "Total Mass: " + str(int(total_weight)) + "kg", HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color.LIGHT_GRAY)
	y += 23
	
	# Display failure status if any elements have yielded or broken
	if broken_count > 0 or yielded_count > 0:
		var status_text = ""
		if broken_count > 0:
			status_text += str(broken_count) + " broken"
		if yielded_count > 0:
			if status_text != "":
				status_text += ", "
			status_text += str(yielded_count) + " yielded"
		draw_string(ThemeDB.fallback_font, Vector2(10, y), "Status: " + status_text, HORIZONTAL_ALIGNMENT_LEFT, -1, 14, Color.ORANGE)

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _input(event):
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_SPACE:
				simulation_running = !simulation_running
			KEY_N:
				placement_mode = "node"
				first_node = null
			KEY_B:
				placement_mode = "beam"
				first_node = null
			KEY_W:
				placement_mode = "weight"
				first_node = null
			KEY_1:
				selected_material = MaterialType.STEEL
			KEY_2:
				selected_material = MaterialType.WOOD
			KEY_3:
				selected_material = MaterialType.ROAD
			KEY_R:
				_reset_simulation()
			KEY_C:
				_clear_all()
			KEY_F:
				_toggle_fixed_node()
			KEY_I:
				_load_bridge(0)
			KEY_O:
				_load_bridge(1)
			KEY_P:
				_load_bridge(2)
			KEY_S:
				_start_stress_recording()
			KEY_T:
				_start_impulse_test()
			KEY_EQUAL, KEY_KP_ADD, KEY_PLUS:
				selected_weight_mass = min(selected_weight_mass + 500, 200000)
			KEY_MINUS, KEY_KP_SUBTRACT:
				selected_weight_mass = max(selected_weight_mass - 500, 500)
			KEY_ENTER:
				_export_to_csv()
	
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_LEFT:
			_handle_click()
		elif event.button_index == MOUSE_BUTTON_RIGHT:
			_handle_right_click()

# ============================================================================
# STRESS RECORDING
# ============================================================================

# Start recording stress data for all elements over time
# Records for 10 seconds at 1200 Hz sampling rate
func _start_stress_recording():
	if not simulation_running:
		print("Start simulation first before recording stress!")
		return
	
	if recording_stress:
		print("Already recording stress data!")
		return
	
	# Initialize recording session
	stress_history.clear()
	recording_stress = true
	stress_recording_elapsed = 0.0
	print("Started stress recording for 10 seconds...")

func _start_impulse_test():
	
	if not simulation_running:
		# Find a non-fixed node near center of bridge
		var center_x = (BRIDGE_SUPPORT_LEFT_X + BRIDGE_SUPPORT_RIGHT_X) / 2.0
		var min_dist = 999999.0
		impulse_test_node = null
		
		for node in nodes:
			if not node.fixed:
				var dist = abs(node.pos.x - center_x)
				if dist < min_dist:
					min_dist = dist
					impulse_test_node = node
		
		# Reset structure to equilibrium
		_reset_simulation()
		
		# Turn off all damping for free vibration
		alpha_damping = 0.0
		beta_damping = 0.0
		
		print("Damping disabled for free vibration test")
		print("Impulse node: ", nodes.find(impulse_test_node))
		print("Impulse force: ", impulse_magnitude, " N")
		
		# Apply impulse as initial velocity (F*dt = m*v)
		var total_mass = impulse_test_node.mass + impulse_test_node.applied_mass
		var impulse_velocity = impulse_magnitude / total_mass
		impulse_test_node.velocity = Vector2(0, impulse_velocity)  # Downward impulse
		
		print("Applied impulse velocity: ", impulse_velocity, " pixels/s")
		
		# Start recording stress immediately
		stress_history.clear()
		recording_stress = true
		stress_recording_elapsed = 0.0
		impulse_test_mode = true
		
		# Start simulation
		simulation_running = true
	else:
		print("Stop simulation first!")
		
func _export_impulse_response():
	"""Export displacement time history for FFT analysis"""
	var datetime = Time.get_datetime_dict_from_system()
	var timestamp = "%04d%02d%02d_%02d%02d%02d" % [datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second]
	var filename = "user://impulse_response_" + timestamp + ".csv"
	
	var file = FileAccess.open(filename, FileAccess.WRITE)
	if file == null:
		print("Error: Could not create file. Error code: ", FileAccess.get_open_error())
		return
	
	var header = "timestep,element"
	file.store_line(header)
	
	# Write displacement data
	for t in range(stress_history.size()):
		var time = t * time_step
		var displacement = stress_history[t][0]
		file.store_line(str(time) + "," + str(displacement))
	
	file.close()
	print("\nImpulse response exported!")
	print("Location: ", ProjectSettings.globalize_path(filename))
	print("\nRun FFT analysis on this data to find natural frequencies")
	print("Peaks in FFT = undamped natural frequencies")
# ============================================================================
# MOUSE INTERACTION HANDLERS
# ============================================================================

# Handle left mouse click based on current placement mode
func _handle_click():
	if placement_mode == "node":
		# Create new node at cursor position
		var new_node = FEMNode.new(snapped_mouse_pos, false)
		nodes.append(new_node)
		
	elif placement_mode == "beam":
		if first_node == null:
			# First click: select starting node
			first_node = _find_nearest_node(snapped_mouse_pos, 20.0)
		else:
			# Second click: create beam to ending node
			var second_node = _find_nearest_node(snapped_mouse_pos, 20.0)
			if second_node != null and second_node != first_node:
				elements.append(FEMElement.new(first_node, second_node, selected_material))
			first_node = null
			
	elif placement_mode == "weight":
		# Add weight to nearest node
		var target_node = _find_nearest_node(snapped_mouse_pos, 30.0)
		if target_node != null:
			target_node.applied_mass += selected_weight_mass
			weights.append(LoadWeight.new(target_node, selected_weight_mass))

# Handle right mouse click - delete operations
func _handle_right_click():
	var nearest_node = _find_nearest_node(snapped_mouse_pos, 20.0)
	if nearest_node != null:
		# First try to remove weights from this node
		for i in range(weights.size() - 1, -1, -1):
			if weights[i].node == nearest_node:
				nearest_node.applied_mass -= weights[i].mass
				weights.remove_at(i)
				return
		
		# If no weights to remove, delete the node and connected elements
		nodes.erase(nearest_node)
		for i in range(elements.size() - 1, -1, -1):
			if elements[i].node_a == nearest_node or elements[i].node_b == nearest_node:
				elements.remove_at(i)

# Toggle whether a node is fixed (pinned to ground)
func _toggle_fixed_node():
	var node = _find_nearest_node(snapped_mouse_pos, 20.0)
	if node != null:
		node.fixed = !node.fixed

# Find the nearest node to a given position
# Returns null if no node within max_dist
func _find_nearest_node(pos: Vector2, max_dist: float) -> FEMNode:
	var nearest: FEMNode = null
	var min_dist = max_dist
	
	for node in nodes:
		var dist = node.pos.distance_to(pos)
		if dist < min_dist:
			min_dist = dist
			nearest = node
	
	return nearest

# ============================================================================
# SIMULATION CONTROL
# ============================================================================

# Reset simulation to initial state without clearing structure
func _reset_simulation():
	simulation_running = false
	for node in nodes:
		node.pos = node.original_pos
		node.velocity = Vector2.ZERO
		node.force = Vector2.ZERO
		
	for element in elements:
		element.broken = false
		element.yielded = false

# Clear everything - structure, weights, and reset state
func _clear_all():
	simulation_running = false
	nodes.clear()
	elements.clear()
	weights.clear()
	first_node = null

# ============================================================================
# DATA EXPORT FUNCTIONS
# ============================================================================

# Export stress history data to CSV file
# Format: timestep, element_0, element_1, ..., element_n
# Each row contains stress values (Pa) for all elements at that timestep
func _export_stress_history():
	var datetime = Time.get_datetime_dict_from_system()
	var timestamp = "%04d%02d%02d_%02d%02d%02d" % [datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second]
	var filename = "user://stress_history_" + timestamp + ".csv"
	
	var file = FileAccess.open(filename, FileAccess.WRITE)
	if file == null:
		print("Error: Could not create stress history file. Error code: ", FileAccess.get_open_error())
		return
	
	# Conversion factor for simulation units to real-world units
	const PIXELS_PER_METER = 10.0
	
	# Write CSV header row
	var header = "timestep"
	for i in range(elements.size()):
		header += ",element_" + str(i)
	file.store_line(header)
	
	# Write stress data for each recorded timestep
	for t in range(stress_history.size()):
		# Time in seconds (each timestep is 1/1200 second)
		var row = str(t * stress_recording_interval)
		var timestep_data = stress_history[t]
		
		# Convert each stress value to Pascals
		for stress_value in timestep_data:
			var stress_real = stress_value * PIXELS_PER_METER
			row += "," + str(stress_real)
		file.store_line(row)
	
	file.close()
	
	print("Stress history exported successfully!")
	print("Location: ", ProjectSettings.globalize_path(filename))
	print("Timesteps recorded: ", stress_history.size())
	print("Elements tracked: ", elements.size())

# Export current structural state to CSV file
# Includes element properties, current stress, strain, and failure status
func _export_to_csv():
	var datetime = Time.get_datetime_dict_from_system()
	var timestamp = "%04d%02d%02d_%02d%02d%02d" % [datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second]
	var filename = "user://bridge_sim_" + timestamp + ".csv"
	
	var file = FileAccess.open(filename, FileAccess.WRITE)
	if file == null:
		print("Error: Could not create CSV file. Error code: ", FileAccess.get_open_error())
		return
	
	# Conversion factor for simulation units to real-world units
	const PIXELS_PER_METER = 10.0
	
	# Write CSV header
	var csv_content = ""
	csv_content += "ID,Material_Type,Yielded,Broken,Stress_Pa,Strain,Total_Force_N,Original_Length_m,Mass_kg,Density_kg_m3,Youngs_Modulus_Pa,Cross_Section_Area_m2,Yield_Strength_Pa,Ultimate_Strength_Pa\n"
	
	# Write data for each element
	for i in range(elements.size()):
		var element = elements[i]
		var props = MATERIAL_PROPERTIES[element.material_type]
		
		# Calculate current mechanical values
		var stress = element.calculate_stress()
		var strain = element.calculate_strain()
		var force = element.calculate_force()
		
		# Convert to real-world SI units
		var original_length_m = element.original_length / PIXELS_PER_METER
		var density_real = props["density"] / PIXELS_PER_METER
		var mass_real = density_real * original_length_m
		var youngs_modulus_real = props["youngs_modulus"] * PIXELS_PER_METER
		var cross_section_area_real = (props["cross_section_area"] / PIXELS_PER_METER) / PIXELS_PER_METER
		var yield_strength_real = props["yield_strength"] * PIXELS_PER_METER
		var ultimate_strength_real = props["ultimate_strength"] * PIXELS_PER_METER
		var stress_real = stress * PIXELS_PER_METER
		var force_real = force / PIXELS_PER_METER
		
		# Build CSV row
		csv_content += "%d,%s,%s,%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % [
			i,
			props["name"],
			element.yielded,
			element.broken,
			stress_real,
			strain,
			force_real,
			original_length_m,
			mass_real,
			density_real,
			youngs_modulus_real,
			cross_section_area_real,
			yield_strength_real,
			ultimate_strength_real
		]
	
	file.store_string(csv_content)
	file.close()
	
	print("CSV exported successfully!")
	print("Location: ", ProjectSettings.globalize_path(filename))
