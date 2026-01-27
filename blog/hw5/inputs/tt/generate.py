for i in range(3, 394):
    mesh=f'<Mesh id="{i}"> \n\
    <Material>2</Material> \n\
    <Transformations>c1 c1</Transformations> \n\
    <Faces plyFile="geometry/mesh_{i:05}.ply" /> \n\
</Mesh>'
    print(mesh)

