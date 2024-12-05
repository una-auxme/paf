# Request Behaviour Change Service

This service is hosted in the node RequestBehaviourChangeService.

Calling it requires a behaviour ID integer (based on the corresponding enum) and returns a bool depending on whether the request is granted.

To use it, import RequestBehaviourChange from planning.srv.

To call it, create a callable instance with:

```python
rospy.wait_for_service('RequestBehaviourChange')

name = rospy.ServiceProxy('RequestBehaviourChange', RequestBehaviourChange)
```

Then, you can just use this instance in a try setup:

```python
try:
    response = name(input)
except rospy.ServiceException as e:
    # handle exception
```

For communication with the behaviour tree this node publishes a granted request to a topic behaviour_request.
