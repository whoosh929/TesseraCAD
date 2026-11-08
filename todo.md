- [x] Bug: When in 2d sketch and I delete a shape, it goes to another sketch for some reason. It should just remain in that 'sketch' until I exit it. If I exit that sketch and it's empty, prompt confirmation from the user that the sketch is empty and thus, will be deleted.
- [x] feature: default mouse navigation: In SOLIDWORKS, the middle mouse button (mouse wheel) is the primary tool for navigating 3D models and sketches.

Default Mouse Wheel Controls

• Zoom: Roll the scroll wheel up or down to zoom in or out centered on your cursor.
• Rotate: Press and hold down the middle mouse button (click the wheel) and drag to rotate your part or assembly.
• Pan: Hold Ctrl + press and hold the middle mouse button, then drag to pan side to side.
• Smooth Zoom: Hold Shift + roll the mouse wheel for finer, smoother zooming.

- [x] feature: add fillets and chambers as features. The fillet or chamber. It needs to reference a model corner edge and inputs some distance value. fillets/cahbmers can also be done in 2d sketch.
- [x] feature: when sketching on a face and a reference geometry is shown, make it much more subtle.
- [x] bug: when in a 2d sketch, pressing esc automatically shifts back to move/select.
- [x] bug: allow dragging an entire geometry in2d sketch mode. This can be done by selecting a geometry and dragging it.
- []feature: add a measure tool
- [x] bug: when a vertex on a model is selected, it's not obvious on the model. Highlight the vertex on the 3d model view.
- [x] feature: allow all dimensions in a 2d sketch to be input using the keyboard.
- [x] feature: 2D sketch dimension constraint locking, in-place double-click editing, and toggling.
- [x] feature: When applying a fillet to a vertex, give the choice for distance or angle.
- [x] feature: when applying a 3d feature like chamfer or extrude, continue to allow for preview of the model as done now, but don't apply the chain until the user confirms. The user confirmation/cancellation should be like you confirm/cancel a 2d sketch .
- [x] bug: when applying a feature like a fillet, not being satisified with it and clicking delete feature, the entire assocaited model gets deleted temporarily and then it comes back. Can you make it so when you delete the feature, it doesn't delete the whole associated model?
- [x] bug: fillet and chamfer features should be childs of the assocaited model.
- [x] bug: when I click a shape in 2d sketch mode, it moves. It should only move if I click and drag.
- [x] feature: add a sublte x/y/z axes reference.
- [x] feature: add a control that recenters view on top, side, and isometric origin.
- [x] bug: when applying a fillet, sometimes the round is inverted, creating a concave shape at a corner instead of a convex.
- [x] bug: propogate fillet corner behavior to chamfers.
- [x] feature: allow creating reference planes and axis from the offset of a selected face of corner face.
- [x] feature: when selecting an edge corner or face, you don't need to give the option to create features on the left side. The shortcut here doens't help that much
- [x] bug: when entering a 2d sketch, the topbar is overloaded with content and it gets cuttoff. Most important is the cancel/confirm sketch button, which should always be on display.
- [] feature: allow 2d sketch dimensions to be in reference from an axis (when one exists).
- [x] bug: when doing an extrusion, the popup window describing the extrusion depth is too small and cannot fit all the text in the popup window. It looks like the 'flip' button is cutoff, specifically. fix this.
- [x] feature: allow extrusions or revolves to cut away material from an existing model (Cut Extrude & Cut Revolve).
- [] feature: convert a model to a 2d drawing
- [] feature: save a model as a stp file.
- [] feature: save a model and retain its model tree for later modification.
- [] bug: selecting edges and models should be reflective of the end-model state. Sometimes when I select edges, it seems to be from the previous feature's model state before the feature was applied.
- [] bug: make extruded and revolved cuts just a toggle of the extruded and revolve feature. Not it's own separate feature.
