# LingTu map annotations and navigation

Vocabulary for describing annotated terrain and navigation without conflating
places, task destinations, and the geometry a robot will follow.

## Language

**Map annotation (地图标注):**
A description attached to a specific saved map that identifies a place or
observed feature, such as a staircase and its entrances. It is not a statement
that every robot can traverse that feature.
_Avoid_: Treating a terrain label as movement permission.

**Traversal rule (通行规则):**
A direction-dependent requirement for traversing an annotated passage, such
as a speed limit, a required body orientation, or a locomotion requirement.
_Avoid_: Treating an arrival action as a rule for movement along a passage.

**Motion-control profile (运动控制模型 / 策略 profile):**
The robot's selected locomotion policy/model and its associated controller
parameters. This is what the operator means by changing the dog's "运动模式"
before stairs. The map stores the required traversal behavior; the selected
RobotConfig's static capability declarations and adapter bind that requirement
to a supported concrete profile. Executor requests the transition before entry
and uses actual active-profile feedback to confirm it. Platform prerequisites,
such as reaching Grounded before switching, belong in entry preparation.
_Avoid_: Product mode, LLM model, route preference, terrain label, or an assumed
vendor-independent stair profile.

**Gait (步态):**
A locomotion pattern or setting exposed by a controller. Changing gait does not
by itself establish that a different motion-control model/profile is active.
_Avoid_: Using a gait command or gait-state number as profile confirmation
without a documented platform mapping.

**Inspection route (巡检路线):**
An ordered collection of destinations and any operations to perform upon
arrival. It does not specify the complete geometry between destinations.
_Avoid_: Execution path.

**Execution path (执行路径):**
The geometric route selected for a particular navigation attempt, including
the movement between destinations.
_Avoid_: Inspection route, persistent map connection.
