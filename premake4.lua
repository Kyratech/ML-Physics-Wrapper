--
-- see https://premake.github.io/ for explanations of these commands.
--
solution ('SimpleBulletWrapper') --Top Level name of the project
   configurations { 'Release' }
      language 'C++' -- The programming language
      project("BulletWrapper") -- The specific name
        kind 'StaticLib' -- the target application
        targetdir('./lib') -- where to put the executable
		links{'libBulletDynamics', 'libBulletCollision', 'libLinearMath'}
        files {"SimpleBulletWrapper/*.cpp"} -- collects all the cpp files in the directory, there must only be one main function.
        buildoptions{'-Wno-write-strings'} -- build option to suppress a very common warning  about strings