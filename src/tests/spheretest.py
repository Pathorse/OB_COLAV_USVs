
   # # Spheres with random data in random positions
   # np.random.seed(0)
   # n_spheres = 10
   # spheres = []
   # for i in range (n_spheres):
   #     spheres.append(
   #         Sphere(
   #             f'Sphere_{i}',
   #             'red',
   #             np.random.randn(2)*10,
   #             np.abs(np.random.rand()*5)
   #         )
   #     )

   # # Plot spheres
   # plt.figure()
   # plt.axis('equal')
   # for sphere in spheres:
   #     sphere.plot_contour()
   # plt.show()
