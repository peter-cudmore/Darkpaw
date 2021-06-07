from core import generate_tables_source, generate_kinematics_source

if __name__ == '__main__':
    header_text, source_text = generate_tables_source()

    with open('../src/gen/tables.h', 'w') as header:
        header.writelines(header_text)

    with open('../src/gen/tables.c', 'w') as source:
        source.writelines(source_text)

    header_text, source_text = generate_kinematics_source()
    with open('../src/gen/model.h') as header:
        header.writelines(header_text)

    with open('../src/gen/model.c', 'w') as source:
        source.writelines(source_text)
