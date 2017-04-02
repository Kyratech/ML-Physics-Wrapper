--
-- see https://premake.github.io/ for explanations of these commands.
--
solution ('SimplePhysicsWrapper') --Top Level name of the project
   configurations { 'Release' }
      language 'C++' -- The programming language
      project("PhysicsWrapper") -- The specific name
        kind 'StaticLib' -- the target application
        targetdir('./lib') -- where to put the executable
		links{'libBulletDynamics', 'libBulletCollision', 'libLinearMath'}
        files {"SimpleBulletWrapper/*.cpp", "SimpleBulletWrapper/*.cc"} -- collects all the cpp and cc files in the directory, there must only be one main function.
        buildoptions{'-Wno-write-strings'} -- build option to suppress a very common warning  about strings