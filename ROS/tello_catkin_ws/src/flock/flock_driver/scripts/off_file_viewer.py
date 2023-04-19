import pandas as pd
import numpy as np
import plotly.graph_objects as go
def read_off(filename):
    """TODO: explain"""
    with open(filename) as off:
        # (1) parse header
        first_line = off.readline()
        if "OFF" not in first_line:
            raise ValueError('The file does not start whith the word OFF')
        color = True if "C" in first_line else False

        count = 1
        for line in off:
            count += 1
            # (2) ignore comments
            if line.startswith("#"):
                continue
            # (3) extract metadata
            line = line.strip().split()
            if len(line) > 1:
                n_vertices = int(line[0])
                n_faces = int(line[1])
                break

        # (4) load data
        data = {}
        point_names = ["x", "y", "z"]
        if color:
            point_names.extend(["red", "green", "blue"])

        data["vertices"] = pd.read_csv(filename, sep=" ", header=None, engine="python",
                                     skiprows=count, skipfooter=n_faces,
                                     names=point_names, index_col=False)
        for n in ["x", "y", "z"]:
            data["vertices"][n] = data["vertices"][n].astype(np.float32)

        if color:
            for n in ["red", "green", "blue"]:
                data["vertices"][n] = data["vertices"][n].astype(np.uint8)

        data["faces"] = pd.read_csv(filename, sep=" ", header=None, engine="python",
                                   skiprows=(count + n_vertices), usecols=[1, 2, 3],
                                   names=["v1", "v2", "v3"])

        return pd.DataFrame.to_numpy(data["vertices"]), pd.DataFrame.to_numpy(data["faces"])

# read_off reads all vertices and faces as numpy arrays
vertices, faces = read_off("pc.off")
x, y, z = vertices[:,0], vertices[:,1], vertices[:,2]
i, j, k = faces[:,0], faces[:,1], faces[:,2]

# Just to make sure I've read everything correctly
#print(f"x:\n{x}\ny:\n{y}\nz:\n{z}\nfaces:\n{faces}")

fig = go.Figure(data=[go.Mesh3d(
        # 8 vertices of a cube
        x=x,
        y=y,
        z=z,
        colorbar_title='z',
        colorscale=[[0, 'gold'],
                    [0.5, 'mediumturquoise'],
                    [1, 'magenta']],
        # Intensity of each vertex, which will be interpolated and color-coded
        intensity = np.linspace(0, 1, 8, endpoint=True),
        # i, j and k give the vertices of triangles
        i = i,
        j = j,
        k = k,
        name='y',
        showscale=True
    )])
fig.show()