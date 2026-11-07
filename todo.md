- [x] Bug: When in 2d sketch and I delete a shape, it goes to another sketch for some reason. It should just remain in that 'sketch' until I exit it. If I exit that sketch and it's empty, prompt confirmation from the user that the sketch is empty and thus, will be deleted.
- [x] feature: default mouse navigation: In SOLIDWORKS, the middle mouse button (mouse wheel) is the primary tool for navigating 3D models and sketches.

Default Mouse Wheel Controls

• Zoom: Roll the scroll wheel up or down to zoom in or out centered on your cursor.
• Rotate: Press and hold down the middle mouse button (click the wheel) and drag to rotate your part or assembly.
• Pan: Hold Ctrl + press and hold the middle mouse button, then drag to pan side to side.
• Smooth Zoom: Hold Shift + roll the mouse wheel for finer, smoother zooming.

- [x] feature: add fillets and chambers as features. The fillet or chamber. It needs to reference a model corner edge and inputs some distance value. fillets/cahbmers can also be done in 2d sketch.
- []feature: when sketching on a face and a reference geometry is shown, make it much more subtle.
- []feature: add a measure tool
- [x] bug: when a vertex on a model is selected, it's not obvious on the model. Highlight the vertex on the 3d model view.
- [] feature: allow all dimensions in a 2d sketch to be input using the keyboard.
- [x] feature: When applying a fillet to a vertex, give the choice for distance or angle.
- [] feature: when applying a 3d feature like chamfer or extrude, continue to allow for preview of the model as done now, but don't apply the chain until the user confirms. The user confirmation/cancellation should be like you confirm/cancel a 2d sketch .
- [x] bug: when applying a feature like a fillet, not being satisified with it and clicking delete feature, the entire assocaited model gets deleted temporarily and then it comes back. Can you make it so when you delete the feature, it doesn't delete the whole associated model?
- [x] bug: fillet and chamfer features should be childs of the assocaited model.
- [] feature: add a sublte x/y/z axes reference.
- [] feature: add a control that recenters view on top, side, and isometric origin.
- [x] bug: when applying a fillet, sometimes the round is inverted, creating a concave shape at a corner instead of a convex.
