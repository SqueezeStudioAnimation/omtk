name = 'omtk'

version = '0.4.20'

requires = ['libSerialization-0.1+']

def commands():
    env.PYTHONPATH.append('{root}')
